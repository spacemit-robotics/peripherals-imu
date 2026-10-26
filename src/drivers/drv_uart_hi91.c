/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file drv_uart_hi91.c
 * @brief Receive-only HI91 UART transport; sensor configuration is external.
 */
#ifndef _DEFAULT_SOURCE
#define _DEFAULT_SOURCE
#endif
#include "imu_core.h"
#include "hi91_parser.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

struct hi91_private {
    char *path;
    uint32_t baud;
    int fd;
    struct hi91_parser parser;
};

static speed_t baud_constant(uint32_t baud) {
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return 0;
    }
}

static int hi91_init(struct imu_dev *dev) {
    struct hi91_private *priv = dev->priv_data;
    struct termios settings;
    if (priv->fd >= 0) return -EALREADY;
    priv->fd = open(priv->path, O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (priv->fd < 0) return -errno;
    if (flock(priv->fd, LOCK_EX | LOCK_NB) < 0 || tcgetattr(priv->fd, &settings) < 0)
        goto fail;
    cfmakeraw(&settings);
    if (cfsetispeed(&settings, baud_constant(priv->baud)) < 0 ||
        cfsetospeed(&settings, baud_constant(priv->baud)) < 0)
        goto fail;
    settings.c_cflag = (settings.c_cflag & ~(CSIZE | CSTOPB | CRTSCTS | PARENB | HUPCL)) |
        CLOCAL | CREAD | CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (tcsetattr(priv->fd, TCSANOW, &settings) < 0 || tcflush(priv->fd, TCIFLUSH) < 0)
        goto fail;
    hi91_parser_init(&priv->parser);
    return 0;
fail: {
    int saved = errno;
    close(priv->fd);
    priv->fd = -1;
    return -saved;
}
}

static int hi91_read(struct imu_dev *dev, struct imu_data *sample) {
    struct hi91_private *priv = dev->priv_data;
    struct imu_data newest;
    int found = 0;
    if (priv->fd < 0) return -ENODEV;
    if (priv->parser.clock_fault) return -EIO;
    for (size_t chunk = 0; chunk < 32; ++chunk) {
        uint8_t bytes[256];
        ssize_t size = read(priv->fd, bytes, sizeof(bytes));
        if (size > 0) {
            struct timespec now;
            int result;
            uint64_t received_us;
            if (clock_gettime(CLOCK_MONOTONIC, &now) < 0) return -errno;
            received_us = (uint64_t)now.tv_sec * 1000000ULL + (uint64_t)now.tv_nsec / 1000ULL;
            result = hi91_parser_feed(&priv->parser, bytes, (size_t)size, received_us, &newest);
            if (result != 0 && result != -EAGAIN) return result;
            if (result == 0) {
                if (found) ++priv->parser.diagnostics.superseded_frames;
                found = 1;
            }
        } else if (size < 0 && errno == EINTR) {
            continue;
        } else if (size == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
            if (!found) return -EAGAIN;
            *sample = newest;
            return 0;
        } else {
            return -errno;
        }
    }
    // Do not return a known backlog as a current sample.
    if (found) ++priv->parser.diagnostics.superseded_frames;
    return -EAGAIN;
}

static int hi91_diagnostics(struct imu_dev *dev, struct imu_diagnostics *diagnostics) {
    struct hi91_private *priv = dev->priv_data;
    *diagnostics = priv->parser.diagnostics;
    return 0;
}

static void hi91_free(struct imu_dev *dev) {
    struct hi91_private *priv = dev->priv_data;
    if (priv->fd >= 0) close(priv->fd);
    free(priv->path);
    free(priv);  // Core owns imu_dev/name/locks; do not free them here.
}

static const struct imu_ops hi91_ops = {
    .init = hi91_init, .read = hi91_read, .get_diagnostics = hi91_diagnostics,
    .free = hi91_free};

static struct imu_dev *hi91_create(void *args) {
    const struct imu_args_uart *uart = args;
    struct imu_dev *dev;
    struct hi91_private *priv;
    if (!uart || !uart->instance || !uart->dev_path || uart->dev_path[0] != '/' ||
        uart->ex_args || !baud_constant(uart->baud))
        return NULL;
    dev = imu_dev_alloc(uart->instance, sizeof(*priv));
    if (!dev) return NULL;
    priv = dev->priv_data;
    priv->fd = -1;
    priv->path = strdup(uart->dev_path);
    if (!priv->path) {
        free(priv);
        free(dev->name);
        free(dev);
        return NULL;
    }
    priv->baud = uart->baud;
    dev->ops = &hi91_ops;
    return dev;
}

REGISTER_IMU_DRIVER("drv_uart_hi91", IMU_DRV_UART, hi91_create);
