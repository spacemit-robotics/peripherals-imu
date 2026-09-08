/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_uart_forsense.c
 * @brief Forsense IMU driver over UART.
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include "imu_core.h"

#define DEG_TO_RAD (3.14159265358979323846f / 180.0f)
#define STANDARD_GRAVITY 9.81f
#define FORSENSE_FRAME_ID 0x0002U
#define FORSENSE_FRAME_PAYLOAD_SIZE 44U
#define FORSENSE_FRAME_SIZE 54U
#define FORSENSE_FRAME_CRC_OFFSET 50U
#define FORSENSE_STREAM_BUFFER_SIZE (FORSENSE_FRAME_SIZE * 4)
#define FORSENSE_MAX_CONTIGUOUS_TIME_GAP_US 60000000U

struct forsense_priv {
    char device[128];
    uint32_t baud;
    int fd;
    uint8_t stream[FORSENSE_STREAM_BUFFER_SIZE];
    size_t stream_size;
    uint32_t last_sensor_time_us;
    uint64_t extended_sensor_time_us;
    int has_sensor_time;
    struct imu_diagnostics diagnostics;
};

enum forsense_decode_result {
    FORSENSE_DECODE_OK = 0,
    FORSENSE_DECODE_FORMAT_ERROR = -1,
    FORSENSE_DECODE_CRC_ERROR = -2,
    FORSENSE_DECODE_VALUE_ERROR = -3,
};

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t read_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static float read_float_le(const uint8_t *data)
{
    uint32_t bits = read_u32_le(data);
    float value;

    memcpy(&value, &bits, sizeof(value));
    return value;
}

static void rpy_to_quaternion(float roll, float pitch, float yaw, float quat[4])
{
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    quat[0] = cr * cp * cy + sr * sp * sy;
    quat[1] = sr * cp * cy - cr * sp * sy;
    quat[2] = cr * sp * cy + sr * cp * sy;
    quat[3] = cr * cp * sy - sr * sp * cy;
}

static uint32_t forsense_crc32(uint32_t crc, const uint8_t *data, uint32_t size)
{
    uint32_t i;

    for (i = 0; i < size; ++i) {
        uint32_t value = crc ^ data[i];
        int bit;

        for (bit = 0; bit < 8; ++bit) {
            value = (value >> 1) ^
                (0xedb88320U & (0U - (value & 1U)));
        }
        crc = value;
    }
    return crc;
}

static int forsense_decode_frame(const uint8_t frame[FORSENSE_FRAME_SIZE],
        struct imu_data *data)
{
    float roll;
    float pitch;
    float yaw;

    if (frame[0] != 0xaa || frame[1] != 0x55 ||
        read_u16_le(frame + 2) != FORSENSE_FRAME_ID ||
        read_u16_le(frame + 4) != FORSENSE_FRAME_PAYLOAD_SIZE) {
        return FORSENSE_DECODE_FORMAT_ERROR;
    }
    if (forsense_crc32(1U, frame, FORSENSE_FRAME_CRC_OFFSET) !=
        read_u32_le(frame + FORSENSE_FRAME_CRC_OFFSET)) {
        return FORSENSE_DECODE_CRC_ERROR;
    }

    memset(data, 0, sizeof(*data));
    data->timestamp_us = read_u32_le(frame + 6);
    pitch = read_float_le(frame + 10) * DEG_TO_RAD;
    roll = read_float_le(frame + 14) * DEG_TO_RAD;
    yaw = read_float_le(frame + 18) * DEG_TO_RAD;
    data->acc[0] = read_float_le(frame + 22) * STANDARD_GRAVITY;
    data->acc[1] = read_float_le(frame + 26) * STANDARD_GRAVITY;
    data->acc[2] = read_float_le(frame + 30) * STANDARD_GRAVITY;
    data->gyro[0] = read_float_le(frame + 34) * DEG_TO_RAD;
    data->gyro[1] = read_float_le(frame + 38) * DEG_TO_RAD;
    data->gyro[2] = read_float_le(frame + 42) * DEG_TO_RAD;
    data->temp = read_float_le(frame + 46);
    if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw) ||
        !isfinite(data->acc[0]) || !isfinite(data->acc[1]) ||
        !isfinite(data->acc[2]) || !isfinite(data->gyro[0]) ||
        !isfinite(data->gyro[1]) || !isfinite(data->gyro[2]) ||
        !isfinite(data->temp)) {
        return FORSENSE_DECODE_VALUE_ERROR;
    }
    rpy_to_quaternion(roll, pitch, yaw, data->quat);
    return 0;
}

static void configure_low_latency(int fd, const char *device)
{
    struct serial_struct serial;

    if (ioctl(fd, TIOCGSERIAL, &serial) != 0)
        return;
    if ((serial.flags & ASYNC_LOW_LATENCY) != 0)
        return;
    serial.flags |= ASYNC_LOW_LATENCY;
    if (ioctl(fd, TIOCSSERIAL, &serial) != 0) {
        fprintf(stderr,
                "[drv_uart_forsense] unable to enable low-latency mode on %s: %s\n",
                device, strerror(errno));
    }
}

static speed_t baud_to_speed(uint32_t baud)
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
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    default:
        return 0;
    }
}

static int configure_uart(int fd, uint32_t baud)
{
    struct termios options;
    const speed_t speed = baud_to_speed(baud);

    if (!speed || tcgetattr(fd, &options) != 0)
        return -1;
    cfmakeraw(&options);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_cflag &= ~PARENB;
    options.c_cflag = (options.c_cflag & ~CSIZE) | CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &options) == 0 ? 0 : -1;
}

static int forsense_init(struct imu_dev *dev)
{
    struct forsense_priv *priv = dev ? dev->priv_data : NULL;

    if (!priv)
        return -1;
    priv->fd = open(priv->device, O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (priv->fd < 0)
        return -1;
    if (configure_uart(priv->fd, priv->baud) < 0) {
        close(priv->fd);
        priv->fd = -1;
        return -1;
    }
    configure_low_latency(priv->fd, priv->device);
    tcflush(priv->fd, TCIFLUSH);
    return 0;
}

static int extend_sensor_timestamp(struct forsense_priv *priv,
        struct imu_data *data)
{
    const uint32_t current = (uint32_t)data->timestamp_us;

    if (!priv->has_sensor_time) {
        priv->last_sensor_time_us = current;
        priv->extended_sensor_time_us = current;
        priv->has_sensor_time = 1;
    } else {
        const uint32_t delta = current - priv->last_sensor_time_us;
        const int moved_backward = current < priv->last_sensor_time_us;

        priv->last_sensor_time_us = current;
        if (moved_backward && delta > FORSENSE_MAX_CONTIGUOUS_TIME_GAP_US)
            ++priv->extended_sensor_time_us;
        else
            priv->extended_sensor_time_us += delta;
    }
    data->timestamp_us = priv->extended_sensor_time_us;
    return 0;
}

static void discard_prefix(struct forsense_priv *priv, size_t size)
{
    if (size >= priv->stream_size) {
        priv->stream_size = 0;
        return;
    }
    memmove(priv->stream, priv->stream + size, priv->stream_size - size);
    priv->stream_size -= size;
}

static int parse_latest_frame(struct forsense_priv *priv, struct imu_data *data)
{
    struct imu_data latest;
    int found = 0;

    while (priv->stream_size >= 2) {
        if (priv->stream[0] != 0xaa || priv->stream[1] != 0x55) {
            ++priv->diagnostics.resync_discarded_bytes;
            discard_prefix(priv, 1);
            continue;
        }
        if (priv->stream_size < FORSENSE_FRAME_SIZE)
            break;
        {
            const int decode_result =
                forsense_decode_frame(priv->stream, &latest);

            if (decode_result == FORSENSE_DECODE_OK &&
                extend_sensor_timestamp(priv, &latest) == 0) {
                if (found)
                    ++priv->diagnostics.superseded_frames;
                ++priv->diagnostics.valid_frames;
                *data = latest;
                found = 1;
                discard_prefix(priv, FORSENSE_FRAME_SIZE);
                continue;
            }
            if (decode_result == FORSENSE_DECODE_CRC_ERROR)
                ++priv->diagnostics.crc_errors;
            else
                ++priv->diagnostics.decode_errors;
            ++priv->diagnostics.resync_discarded_bytes;
            discard_prefix(priv, 1);
        }
    }
    return found ? 0 : -1;
}

static int forsense_read(struct imu_dev *dev, struct imu_data *data)
{
    struct forsense_priv *priv = dev ? dev->priv_data : NULL;
    uint8_t incoming[128];
    ssize_t size;

    if (!priv || !data || priv->fd < 0)
        return -1;
    for (;;) {
        size = read(priv->fd, incoming, sizeof(incoming));
        if (size > 0) {
            size_t copy_size = (size_t)size;
            if (copy_size > sizeof(priv->stream) - priv->stream_size) {
                size_t discard = copy_size - (sizeof(priv->stream) - priv->stream_size);
                priv->diagnostics.overflow_discarded_bytes += discard;
                discard_prefix(priv, discard);
            }
            memcpy(priv->stream + priv->stream_size, incoming, copy_size);
            priv->stream_size += copy_size;
            continue;
        }
        if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
            return -1;
        break;
    }
    if (parse_latest_frame(priv, data) < 0)
        return -1;
    return 0;
}

static int forsense_get_diagnostics(struct imu_dev *dev,
        struct imu_diagnostics *diagnostics)
{
    struct forsense_priv *priv = dev ? dev->priv_data : NULL;

    if (!priv || !diagnostics)
        return -1;
    *diagnostics = priv->diagnostics;
    return 0;
}

static void forsense_free(struct imu_dev *dev)
{
    struct forsense_priv *priv;

    if (!dev)
        return;
    priv = dev->priv_data;
    if (priv && priv->fd >= 0)
        close(priv->fd);
    free(dev->priv_data);
    free(dev->name);
    free(dev);
}

static const struct imu_ops forsense_ops = {
    .init = forsense_init,
    .read = forsense_read,
    .get_diagnostics = forsense_get_diagnostics,
    .free = forsense_free,
};

static struct imu_dev *forsense_create(void *args)
{
    const struct imu_args_uart *uart_args = args;
    struct imu_dev *dev;
    struct forsense_priv *priv;

    if (!uart_args || !uart_args->instance || !uart_args->dev_path ||
        !baud_to_speed(uart_args->baud))
        return NULL;
    dev = imu_dev_alloc(uart_args->instance, sizeof(*priv));
    if (!dev)
        return NULL;
    priv = dev->priv_data;
    snprintf(priv->device, sizeof(priv->device), "%s", uart_args->dev_path);
    priv->baud = uart_args->baud;
    priv->fd = -1;
    dev->ops = &forsense_ops;
    return dev;
}

REGISTER_IMU_DRIVER("drv_uart_forsense", IMU_DRV_UART, forsense_create);
