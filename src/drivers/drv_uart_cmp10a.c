/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * CMP10A IMU UART Driver
 * Protocol: 0x55 + type + 8 bytes data + checksum
 * Types: 0x51 (accel), 0x52 (gyro), 0x53 (angle), 0x54 (mag)
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

#include "../imu_core.h"

#define CMP10A_FRAME_LEN      11
#define CMP10A_HEADER         0x55
#define CMP10A_TYPE_ACCEL     0x51
#define CMP10A_TYPE_GYRO      0x52
#define CMP10A_TYPE_ANGLE     0x53
#define CMP10A_TYPE_MAG       0x54

#define CMP10A_ACCEL_SCALE    (16.0f * 9.8f / 32768.0f)   /* m/s^2 */
#define CMP10A_GYRO_SCALE     (2000.0f * M_PI / 180.0f / 32768.0f)  /* rad/s */
#define CMP10A_ANGLE_SCALE    (180.0f / 32768.0f)         /* degrees */

struct cmp10a_priv {
    char dev_path[64];
    uint32_t baud;
    int fd;

    /* ring buffer for parsing */
    uint8_t buffer[CMP10A_FRAME_LEN];
    int buf_idx;

    /* parsed sensor data */
    float acceleration[3];   /* m/s^2 */
    float angular_velocity[3]; /* rad/s */
    float angle_degree[3];   /* degrees */
    float magnetometer[3];

    /* data ready flags */
    uint8_t accel_ready;
    uint8_t gyro_ready;
    uint8_t angle_ready;
};

static uint64_t get_timestamp_us(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static speed_t baud_to_speed(uint32_t baud)
{
    switch (baud) {
    case 9600:   return B9600;
    case 19200:  return B19200;
    case 38400:  return B38400;
    case 57600:  return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    default:     return B9600;
    }
}

static int16_t bytes_to_short(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)high << 8 | (uint16_t)low);
}

static uint8_t calc_checksum(const uint8_t *data, int len)
{
    uint32_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum & 0xFF);
}

static int cmp10a_parse_frame(struct cmp10a_priv *priv, const uint8_t *frame)
{
    uint8_t type = frame[1];
    uint8_t checksum = frame[10];

    /* verify checksum */
    if (calc_checksum(frame, 10) != checksum) {
        printf("[CMP10A] checksum error for type 0x%02X\n", type);
        return -1;
    }

    /* parse data based on type */
    int16_t raw[4];
    raw[0] = bytes_to_short(frame[2], frame[3]);
    raw[1] = bytes_to_short(frame[4], frame[5]);
    raw[2] = bytes_to_short(frame[6], frame[7]);
    raw[3] = bytes_to_short(frame[8], frame[9]);

    switch (type) {
    case CMP10A_TYPE_ACCEL:
        priv->acceleration[0] = raw[0] * CMP10A_ACCEL_SCALE;
        priv->acceleration[1] = raw[1] * CMP10A_ACCEL_SCALE;
        priv->acceleration[2] = raw[2] * CMP10A_ACCEL_SCALE;
        priv->accel_ready = 1;
        break;

    case CMP10A_TYPE_GYRO:
        priv->angular_velocity[0] = raw[0] * CMP10A_GYRO_SCALE;
        priv->angular_velocity[1] = raw[1] * CMP10A_GYRO_SCALE;
        priv->angular_velocity[2] = raw[2] * CMP10A_GYRO_SCALE;
        priv->gyro_ready = 1;
        break;

    case CMP10A_TYPE_ANGLE:
        priv->angle_degree[0] = raw[0] * CMP10A_ANGLE_SCALE;
        priv->angle_degree[1] = raw[1] * CMP10A_ANGLE_SCALE;
        priv->angle_degree[2] = raw[2] * CMP10A_ANGLE_SCALE;
        priv->angle_ready = 1;
        break;

    case CMP10A_TYPE_MAG:
        priv->magnetometer[0] = (float)raw[0];
        priv->magnetometer[1] = (float)raw[1];
        priv->magnetometer[2] = (float)raw[2];
        break;

    default:
        return -1;
    }

    return 0;
}

static int cmp10a_process_byte(struct cmp10a_priv *priv, uint8_t byte)
{
    int ret = 0;

    priv->buffer[priv->buf_idx] = byte;
    priv->buf_idx++;

    /* check header */
    if (priv->buffer[0] != CMP10A_HEADER) {
        priv->buf_idx = 0;
        return 0;
    }

    /* wait for complete frame */
    if (priv->buf_idx < CMP10A_FRAME_LEN) {
        return 0;
    }

    /* parse complete frame */
    ret = cmp10a_parse_frame(priv, priv->buffer);
    priv->buf_idx = 0;

    return ret;
}

static void euler_to_quaternion(float roll, float pitch, float yaw, float *quat)
{
    /* Convert Euler angles (radians) to quaternion [w, x, y, z] */
    float cr = cosf(roll / 2.0f);
    float sr = sinf(roll / 2.0f);
    float cp = cosf(pitch / 2.0f);
    float sp = sinf(pitch / 2.0f);
    float cy = cosf(yaw / 2.0f);
    float sy = sinf(yaw / 2.0f);

    quat[0] = cr * cp * cy + sr * sp * sy;  /* w */
    quat[1] = sr * cp * cy - cr * sp * sy;  /* x */
    quat[2] = cr * sp * cy + sr * cp * sy;  /* y */
    quat[3] = cr * cp * sy - sr * sp * cy;  /* z */
}

static int cmp10a_init(struct imu_dev *dev)
{
    struct cmp10a_priv *priv = dev->priv_data;
    struct termios tty;
    speed_t speed;

    printf("[CMP10A] init: %s, dev=%s, baud=%u\n",
            dev->name, priv->dev_path, priv->baud);

    /* open serial port */
    priv->fd = open(priv->dev_path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (priv->fd < 0) {
        printf("[CMP10A] failed to open %s: %s\n",
                priv->dev_path, strerror(errno));
        return -1;
    }

    /* configure serial port */
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(priv->fd, &tty) != 0) {
        printf("[CMP10A] tcgetattr failed: %s\n", strerror(errno));
        close(priv->fd);
        priv->fd = -1;
        return -1;
    }

    speed = baud_to_speed(priv->baud);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    /* 8N1, no flow control */
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    /* raw input */
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

    /* raw output */
    tty.c_oflag &= ~OPOST;

    /* non-blocking read */
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;  /* 100ms timeout */

    if (tcsetattr(priv->fd, TCSANOW, &tty) != 0) {
        printf("[CMP10A] tcsetattr failed: %s\n", strerror(errno));
        close(priv->fd);
        priv->fd = -1;
        return -1;
    }

    /* flush buffers */
    tcflush(priv->fd, TCIOFLUSH);

    printf("[CMP10A] serial port opened successfully\n");
    return 0;
}

static int cmp10a_read(struct imu_dev *dev, struct imu_data *data)
{
    struct cmp10a_priv *priv = dev->priv_data;
    uint8_t buf[64];
    ssize_t n;
    int timeout_ms = 100;
    int elapsed_ms = 0;

    if (priv->fd < 0) {
        return -1;
    }

    /* clear ready flags */
    priv->accel_ready = 0;
    priv->gyro_ready = 0;
    priv->angle_ready = 0;

    /* read and parse until we have a complete set of data or timeout */
    while (elapsed_ms < timeout_ms) {
        n = read(priv->fd, buf, sizeof(buf));
        if (n > 0) {
            for (ssize_t i = 0; i < n; i++) {
                cmp10a_process_byte(priv, buf[i]);
            }

            /* check if we have all required data */
            if (priv->accel_ready && priv->gyro_ready && priv->angle_ready) {
                break;
            }
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            printf("[CMP10A] read error: %s\n", strerror(errno));
            return -1;
        }

        usleep(1000);
        elapsed_ms++;
    }

    /* fill output data structure */
    data->timestamp_us = get_timestamp_us();

    data->acc[0] = priv->acceleration[0];
    data->acc[1] = priv->acceleration[1];
    data->acc[2] = priv->acceleration[2];

    data->gyro[0] = priv->angular_velocity[0];
    data->gyro[1] = priv->angular_velocity[1];
    data->gyro[2] = priv->angular_velocity[2];

    data->mag[0] = priv->magnetometer[0];
    data->mag[1] = priv->magnetometer[1];
    data->mag[2] = priv->magnetometer[2];

    /* convert angle to quaternion */
    float roll_rad = priv->angle_degree[0] * (float)M_PI / 180.0f;
    float pitch_rad = priv->angle_degree[1] * (float)M_PI / 180.0f;
    float yaw_rad = priv->angle_degree[2] * (float)M_PI / 180.0f;
    euler_to_quaternion(roll_rad, pitch_rad, yaw_rad, data->quat);

    data->temp = 0.0f;  /* temperature not available in this protocol */

    return 0;
}

static void cmp10a_free(struct imu_dev *dev)
{
    struct cmp10a_priv *priv;

    if (!dev)
        return;

    printf("[CMP10A] free: %s\n", dev->name);

    priv = dev->priv_data;
    if (priv && priv->fd >= 0) {
        close(priv->fd);
        priv->fd = -1;
    }

    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free((void *)dev->name);
    free(dev);
}

static const struct imu_ops cmp10a_ops = {
    .init = cmp10a_init,
    .read = cmp10a_read,
    .free = cmp10a_free,
};

static struct imu_dev *cmp10a_create(void *args)
{
    struct imu_args_uart *a = (struct imu_args_uart *)args;
    struct imu_dev *dev;
    struct cmp10a_priv *priv;

    if (!a || !a->instance || !a->dev_path)
        return NULL;

    dev = imu_dev_alloc(a->instance, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = dev->priv_data;
    dev->ops = &cmp10a_ops;

    strncpy(priv->dev_path, a->dev_path, sizeof(priv->dev_path) - 1);
    priv->dev_path[sizeof(priv->dev_path) - 1] = '\0';
    priv->baud = a->baud ? a->baud : 9600;  /* default 9600 baud */
    priv->fd = -1;
    priv->buf_idx = 0;

    memset(priv->buffer, 0, sizeof(priv->buffer));
    memset(priv->acceleration, 0, sizeof(priv->acceleration));
    memset(priv->angular_velocity, 0, sizeof(priv->angular_velocity));
    memset(priv->angle_degree, 0, sizeof(priv->angle_degree));
    memset(priv->magnetometer, 0, sizeof(priv->magnetometer));

    return dev;
}

REGISTER_IMU_DRIVER("CMP10A", IMU_DRV_UART, cmp10a_create);
