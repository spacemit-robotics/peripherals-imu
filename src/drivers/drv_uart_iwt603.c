#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif

/*
 * Copyright (C) 2025 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu_core.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <pthread.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#define IWT603_FRAME_SIZE 11U
#define IWT603_BUFFER_SIZE 512U
#define IWT603_PATH_CHECK_US 100000U
#define IWT603_RECONNECT_US 200000U
#define IWT603_PI 3.14159265358979323846f
#define IWT603_ACCEL_CORRECTION_GAIN 1.0f
#define IWT603_MAX_ACCEL_AGE_US 100000ULL

struct iwt603_priv {
    pthread_mutex_t mutex;
    unsigned event_pending;
    int initialized;
    char device[256];
    uint32_t baud;
    int fd;
    int event_fd;
    uint8_t buffer[IWT603_BUFFER_SIZE];
    size_t used;
    struct imu_data data;
    uint64_t acceleration_timestamp_us;
    uint64_t gyro_timestamp_us;
    uint64_t attitude_timestamp_us;
    uint64_t next_path_check_us;
    uint64_t next_reconnect_us;
    int has_acceleration;
    int has_gyro;
    int has_attitude;
};

static uint64_t monotonic_us(void)
{
    struct timespec now;

    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
        return 0;
    return (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_nsec / 1000ULL;
}

static int16_t read_word(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static speed_t baud_constant(uint32_t baud)
{
    switch (baud) {
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 921600:
        return B921600;
    default:
        return 0;
    }
}

static void close_port(struct iwt603_priv *priv)
{
    if (priv->fd >= 0)
        close(priv->fd);
    priv->fd = -1;
    priv->used = 0;
    priv->has_acceleration = 0;
    priv->has_gyro = 0;
    priv->has_attitude = 0;
    priv->acceleration_timestamp_us = 0;
    priv->gyro_timestamp_us = 0;
    priv->attitude_timestamp_us = 0;
    priv->event_pending = 0;
    memset(&priv->data, 0, sizeof(priv->data));
}

static int open_port(struct iwt603_priv *priv)
{
    struct termios options;
    speed_t speed = baud_constant(priv->baud);
    int fd;

    if (!speed)
        return -EINVAL;
    fd = open(priv->device, O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0)
        return -errno;
    if (tcgetattr(fd, &options) != 0) {
        int error = errno;
        close(fd);
        return -error;
    }
    cfmakeraw(&options);
    if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0) {
        int error = errno;

        close(fd);
        return -error;
    }
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB | CSIZE);
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        int error = errno;
        close(fd);
        return -error;
    }
    tcflush(fd, TCIFLUSH);
    priv->fd = fd;
    priv->used = 0;
    priv->next_path_check_us = monotonic_us() + IWT603_PATH_CHECK_US;
    return 0;
}

static int path_still_matches(const struct iwt603_priv *priv)
{
    struct stat opened;
    struct stat configured;
    char fd_path[PATH_MAX];
    char opened_path[PATH_MAX];
    char configured_path[PATH_MAX];

    if (priv->fd < 0 || fstat(priv->fd, &opened) != 0 || stat(priv->device, &configured) != 0 ||
        !S_ISCHR(opened.st_mode) || !S_ISCHR(configured.st_mode) || opened.st_rdev != configured.st_rdev)
        return 0;
    if (!realpath(priv->device, configured_path) ||
        snprintf(fd_path, sizeof(fd_path), "/proc/self/fd/%d", priv->fd) >= (int)sizeof(fd_path))
        return 0;
    ssize_t length = readlink(fd_path, opened_path, sizeof(opened_path) - 1U);
    if (length < 0 || (size_t)length >= sizeof(opened_path))
        return 0;
    opened_path[length] = '\0';
    return strcmp(opened_path, configured_path) == 0;
}

static int checksum_valid(const uint8_t *frame)
{
    uint8_t sum = 0;
    size_t i;

    for (i = 0; i < IWT603_FRAME_SIZE - 1U; ++i)
        sum = (uint8_t)(sum + frame[i]);
    return sum == frame[IWT603_FRAME_SIZE - 1U];
}

static void parse_frame(struct iwt603_priv *priv, const uint8_t *frame)
{
    const int16_t x = read_word(frame + 2);
    const int16_t y = read_word(frame + 4);
    const int16_t z = read_word(frame + 6);
    const int16_t extra = read_word(frame + 8);
    const uint64_t timestamp = monotonic_us();

    if (frame[1] == 0x51) {
        priv->event_pending |= 1U;
        priv->data.acc[0] = x / 32768.0f * 16.0f * 9.80665f;
        priv->data.acc[1] = y / 32768.0f * 16.0f * 9.80665f;
        priv->data.acc[2] = z / 32768.0f * 16.0f * 9.80665f;
        priv->data.temp = extra / 100.0f;
        priv->acceleration_timestamp_us = timestamp;
        priv->has_acceleration = 1;
    } else if (frame[1] == 0x52) {
        priv->event_pending |= 2U;
        const float scale = 2000.0f * IWT603_PI / 180.0f / 32768.0f;
        priv->data.gyro[0] = x * scale;
        priv->data.gyro[1] = y * scale;
        priv->data.gyro[2] = z * scale;
        priv->gyro_timestamp_us = timestamp;
        priv->has_gyro = 1;
    } else if (frame[1] == 0x59) {
        /* WitMotion/IWT603 quaternion packet: q0,q1,q2,q3 = w,x,y,z,
         * each int16 / 32768. */
        {
            const int16_t q0 = read_word(frame + 2);
            const int16_t q1 = read_word(frame + 4);
            const int16_t q2 = read_word(frame + 6);
            const int16_t q3 = read_word(frame + 8);
            priv->data.quat[0] = (float)q0 / 32768.0f;
            priv->data.quat[1] = (float)q1 / 32768.0f;
            priv->data.quat[2] = (float)q2 / 32768.0f;
            priv->data.quat[3] = (float)q3 / 32768.0f;
            float norm = sqrtf(priv->data.quat[0] * priv->data.quat[0] + priv->data.quat[1] * priv->data.quat[1] +
                priv->data.quat[2] * priv->data.quat[2] + priv->data.quat[3] * priv->data.quat[3]);
            if (!isfinite(norm) || norm < 1.0e-6f) {
                memset(priv->data.quat, 0, sizeof(priv->data.quat));
                return;
            }
            for (int i = 0; i < 4; ++i)
                priv->data.quat[i] /= norm;
            priv->attitude_timestamp_us = timestamp;
            priv->has_attitude = 1;
        }
    }
}

static void process_buffer(struct iwt603_priv *priv)
{
    size_t offset = 0;

    while (priv->used - offset >= IWT603_FRAME_SIZE) {
        while (offset < priv->used && priv->buffer[offset] != 0x55)
            ++offset;
        if (priv->used - offset < IWT603_FRAME_SIZE)
            break;
        if (!checksum_valid(priv->buffer + offset)) {
            ++offset;
            continue;
        }
        parse_frame(priv, priv->buffer + offset);
        offset += IWT603_FRAME_SIZE;
    }
    if (offset) {
        memmove(priv->buffer, priv->buffer + offset, priv->used - offset);
        priv->used -= offset;
    }
    if (priv->used == sizeof(priv->buffer))
        priv->used = 0;
}

static int iwt603_init(struct imu_dev *dev)
{
    struct iwt603_priv *priv = dev->priv_data;
    int result;

    if (pthread_mutex_lock(&priv->mutex) != 0)
        return -EIO;
    close_port(priv);
    result = open_port(priv);
    priv->initialized = result == 0;
    (void)pthread_mutex_unlock(&priv->mutex);
    return result;
}

static int maintain_port(struct iwt603_priv *priv)
{
    uint64_t now = monotonic_us();

    if (priv->fd >= 0 && now >= priv->next_path_check_us) {
        priv->next_path_check_us = now + IWT603_PATH_CHECK_US;
        if (!path_still_matches(priv)) {
            close_port(priv);
            priv->next_reconnect_us = now;
        }
    }
    if (priv->fd < 0) {
        if (now < priv->next_reconnect_us)
            return -EAGAIN;
        priv->next_reconnect_us = now + IWT603_RECONNECT_US;
        if (open_port(priv) < 0)
            return -EAGAIN;
    }

    return 0;
}

static int iwt603_read_locked(struct iwt603_priv *priv, struct imu_data *data, int event)
{
    uint64_t now = monotonic_us();
    if (!event && maintain_port(priv) < 0)
        return -EAGAIN;
    int fd = event ? priv->event_fd : priv->fd;
    if (event && fd < 0)
        return -ENODEV;
    /* Bound one drain so a continuously streaming UART cannot starve stop. */
    for (unsigned int batch = 0; batch < 16 && priv->used < sizeof(priv->buffer); ++batch) {
        ssize_t count = read(fd, priv->buffer + priv->used,
            event ? IWT603_FRAME_SIZE - priv->used : sizeof(priv->buffer) - priv->used);
        if (count > 0) {
            priv->used += (size_t)count;
            process_buffer(priv);
            if (event && priv->event_pending == 3U)
                break;
            continue;
        }
        if (count == 0)
            break;
        if (errno == EINTR)
            continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            break;
        if (event)
            return -errno;
        close_port(priv);
        priv->next_reconnect_us = now + IWT603_RECONNECT_US;
        return -EAGAIN;
    }
    now = monotonic_us();
    if (!priv->has_acceleration || !priv->has_gyro || now < priv->acceleration_timestamp_us ||
        now - priv->acceleration_timestamp_us > IWT603_MAX_ACCEL_AGE_US || now < priv->gyro_timestamp_us ||
        now - priv->gyro_timestamp_us > IWT603_MAX_ACCEL_AGE_US)
        return -EAGAIN;
    priv->data.timestamp_us = priv->gyro_timestamp_us;
    if (!event && priv->has_attitude && priv->attitude_timestamp_us > priv->data.timestamp_us)
        priv->data.timestamp_us = priv->attitude_timestamp_us;
    if (!priv->has_attitude || now < priv->attitude_timestamp_us ||
        now - priv->attitude_timestamp_us > IWT603_MAX_ACCEL_AGE_US)
        memset(priv->data.quat, 0, sizeof(priv->data.quat));
    *data = priv->data;
    return 0;
}

static int iwt603_read(struct imu_dev *dev, struct imu_data *data)
{
    struct iwt603_priv *priv = dev->priv_data;
    int result;

    if (pthread_mutex_lock(&priv->mutex) != 0)
        return -EIO;
    result = iwt603_read_locked(priv, data, 0);
    (void)pthread_mutex_unlock(&priv->mutex);
    return result;
}

static int iwt603_event_start(struct imu_dev *dev, struct imu_event_source *source)
{
    struct iwt603_priv *priv = dev->priv_data;
    int result = pthread_mutex_lock(&priv->mutex);
    if (result != 0)
        return -EIO;
    if (!priv->initialized || !path_still_matches(priv)) {
        (void)pthread_mutex_unlock(&priv->mutex);
        return -ENODEV;
    }
    priv->event_fd = fcntl(priv->fd, F_DUPFD_CLOEXEC, 0);
    if (priv->event_fd < 0) {
        result = -errno;
        (void)pthread_mutex_unlock(&priv->mutex);
        return result;
    }
    priv->event_pending = 0;
    source->fd = priv->event_fd;
    source->events = POLLIN;
    (void)pthread_mutex_unlock(&priv->mutex);
    return 0;
}

static int iwt603_event_read(struct imu_dev *dev, struct imu_data *data)
{
    struct iwt603_priv *priv = dev->priv_data;
    int result;

    if (pthread_mutex_lock(&priv->mutex) != 0)
        return -EIO;
    result = iwt603_read_locked(priv, data, 1);
    /* Require a fresh accel/gyro pair, accumulated across fragmented UART events. */
    if (!result && priv->event_pending != 3U)
        result = -EAGAIN;
    else if (!result)
        priv->event_pending = 0;
    (void)pthread_mutex_unlock(&priv->mutex);
    return result;
}

static void iwt603_event_stop(struct imu_dev *dev)
{
    struct iwt603_priv *priv = dev->priv_data;

    if (pthread_mutex_lock(&priv->mutex) != 0)
        return;
    if (priv->event_fd >= 0)
        close(priv->event_fd);
    priv->event_fd = -1;
    (void)pthread_mutex_unlock(&priv->mutex);
}

static void iwt603_free(struct imu_dev *dev)
{
    if (!dev)
        return;
    if (dev->priv_data) {
        struct iwt603_priv *priv = dev->priv_data;
        close_port(priv);
        (void)pthread_mutex_destroy(&priv->mutex);
        free(priv);
    }
}

static const struct imu_ops iwt603_ops = {
    .init = iwt603_init,
    .read = iwt603_read,
    .event_start = iwt603_event_start,
    .event_read = iwt603_event_read,
    .event_stop = iwt603_event_stop,
    .free = iwt603_free,
};

static struct imu_dev *iwt603_create(void *args)
{
    const struct imu_args_uart *uart_args = args;
    struct imu_dev *dev;
    struct iwt603_priv *priv;

    if (!uart_args || !uart_args->instance || !uart_args->dev_path || !uart_args->baud)
        return NULL;
    dev = imu_dev_alloc(uart_args->instance, sizeof(*priv));
    if (!dev)
        return NULL;
    priv = dev->priv_data;
    priv->fd = -1;
    priv->event_fd = -1;
    if (pthread_mutex_init(&priv->mutex, NULL) != 0) {
        imu_free(dev);
        return NULL;
    }
    dev->ops = &iwt603_ops;
    if (strlen(uart_args->dev_path) >= sizeof(priv->device)) {
        imu_free(dev);
        return NULL;
    }
    strcpy(priv->device, uart_args->dev_path);
    priv->baud = uart_args->baud;
    priv->fd = -1;
    return dev;
}

REGISTER_IMU_DRIVER("drv_uart_iwt603", IMU_DRV_UART, iwt603_create)
