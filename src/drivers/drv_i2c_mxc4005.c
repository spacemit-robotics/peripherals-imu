/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "../imu_core.h"

#define IIO_SYSFS_ROOT "/sys/bus/iio/devices"

struct mxc4005_priv {
    char device_dir[128];
    char accel_x_path[160];
    char accel_y_path[160];
    char accel_z_path[160];
};

static uint64_t get_timestamp_us(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static int is_decimal_string(const char *str)
{
    size_t i;

    if (!str || !str[0])
        return 0;

    for (i = 0; str[i] != '\0'; i++) {
        if (str[i] < '0' || str[i] > '9')
            return 0;
    }

    return 1;
}

static int read_int_from_file(const char *path, int *out_value)
{
    char buf[32];
    char *end = NULL;
    FILE *fp;
    int64_t val;

    if (!path || !out_value)
        return -EINVAL;

    fp = fopen(path, "r");
    if (!fp)
        return -errno;

    if (!fgets(buf, sizeof(buf), fp)) {
        int err = ferror(fp) ? errno : EIO;

        fclose(fp);
        return -err;
    }

    fclose(fp);

    val = strtoll(buf, &end, 10);
    if (end == buf)
        return -EINVAL;

    *out_value = (int)val;
    return 0;
}

static int fill_axis_path(char *dst, size_t dst_sz, const char *device_dir,
        const char *axis)
{
    int n;

    n = snprintf(dst, dst_sz, "%s/in_accel_%s_raw", device_dir, axis);
    if (n < 0 || (size_t)n >= dst_sz)
        return -EINVAL;

    return 0;
}

static int resolve_device_dir(char *dst, size_t dst_sz, const char *dev_path,
        uint8_t device_index)
{
    int n;

    if (dev_path && dev_path[0]) {
        if (strncmp(dev_path, IIO_SYSFS_ROOT "/", strlen(IIO_SYSFS_ROOT "/")) == 0) {
            n = snprintf(dst, dst_sz, "%s", dev_path);
        } else if (strncmp(dev_path, "iio:device", strlen("iio:device")) == 0) {
            n = snprintf(dst, dst_sz, "%s/%s", IIO_SYSFS_ROOT, dev_path);
        } else if (is_decimal_string(dev_path)) {
            n = snprintf(dst, dst_sz, "%s/iio:device%s", IIO_SYSFS_ROOT, dev_path);
        } else {
            return -EINVAL;
        }
    } else {
        n = snprintf(dst, dst_sz, "%s/iio:device%u", IIO_SYSFS_ROOT,
                (unsigned int)device_index);
    }

    if (n < 0 || (size_t)n >= dst_sz)
        return -EINVAL;

    return 0;
}

static int mxc4005_init(struct imu_dev *dev)
{
    struct mxc4005_priv *priv = dev->priv_data;

    printf("[MXC4005] init: %s\n", dev->name);
    printf("[MXC4005] device_dir=%s\n", priv->device_dir);

    if (access(priv->accel_x_path, R_OK) != 0) {
        printf("[MXC4005] missing node: %s\n", priv->accel_x_path);
        return -errno;
    }
    if (access(priv->accel_y_path, R_OK) != 0) {
        printf("[MXC4005] missing node: %s\n", priv->accel_y_path);
        return -errno;
    }
    if (access(priv->accel_z_path, R_OK) != 0) {
        printf("[MXC4005] missing node: %s\n", priv->accel_z_path);
        return -errno;
    }

    return 0;
}

static int mxc4005_read(struct imu_dev *dev, struct imu_data *data)
{
    struct mxc4005_priv *priv = dev->priv_data;
    int raw_x = 0;
    int raw_y = 0;
    int raw_z = 0;
    int ret;

    memset(data, 0, sizeof(*data));

    ret = read_int_from_file(priv->accel_x_path, &raw_x);
    if (ret < 0)
        return ret;

    ret = read_int_from_file(priv->accel_y_path, &raw_y);
    if (ret < 0)
        return ret;

    ret = read_int_from_file(priv->accel_z_path, &raw_z);
    if (ret < 0)
        return ret;

    data->timestamp_us = get_timestamp_us();
    data->acc[0] = (float)raw_x;
    data->acc[1] = (float)raw_y;
    data->acc[2] = (float)raw_z;

    return 0;
}

static void mxc4005_free(struct imu_dev *dev)
{
    if (!dev)
        return;

    printf("[MXC4005] free: %s\n", dev->name);

    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free((void *)dev->name);
    free(dev);
}

static const struct imu_ops mxc4005_ops = {
    .init = mxc4005_init,
    .read = mxc4005_read,
    .free = mxc4005_free,
};

static struct imu_dev *mxc4005_create(void *args)
{
    struct imu_args_i2c *a = (struct imu_args_i2c *)args;
    struct imu_dev *dev;
    struct mxc4005_priv *priv;

    if (!a || !a->instance)
        return NULL;

    dev = imu_dev_alloc(a->instance, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = dev->priv_data;
    dev->ops = &mxc4005_ops;

    if (resolve_device_dir(priv->device_dir, sizeof(priv->device_dir),
            a->dev_path, a->addr) < 0) {
        mxc4005_free(dev);
        return NULL;
    }

    if (fill_axis_path(priv->accel_x_path, sizeof(priv->accel_x_path),
            priv->device_dir, "x") < 0) {
        mxc4005_free(dev);
        return NULL;
    }

    if (fill_axis_path(priv->accel_y_path, sizeof(priv->accel_y_path),
            priv->device_dir, "y") < 0) {
        mxc4005_free(dev);
        return NULL;
    }

    if (fill_axis_path(priv->accel_z_path, sizeof(priv->accel_z_path),
            priv->device_dir, "z") < 0) {
        mxc4005_free(dev);
        return NULL;
    }

    return dev;
}

REGISTER_IMU_DRIVER("mxc4005", IMU_DRV_I2C, mxc4005_create);
