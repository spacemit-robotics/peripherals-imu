/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "imu_core.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

/* matrix-vector multiply: out = R * in */
static void mat3_mult_vec3(const float R[9], const float in[3], float out[3])
{
    out[0] = R[0] * in[0] + R[1] * in[1] + R[2] * in[2];
    out[1] = R[3] * in[0] + R[4] * in[1] + R[5] * in[2];
    out[2] = R[6] * in[0] + R[7] * in[1] + R[8] * in[2];
}

void imu_apply_rotation_and_offset(struct imu_dev *dev, struct imu_data *data)
{
    float temp_acc[3], temp_gyro[3];

    /* subtract bias offset */
    data->acc[0] -= dev->config.acc_offset[0];
    data->acc[1] -= dev->config.acc_offset[1];
    data->acc[2] -= dev->config.acc_offset[2];

    data->gyro[0] -= dev->config.gyro_offset[0];
    data->gyro[1] -= dev->config.gyro_offset[1];
    data->gyro[2] -= dev->config.gyro_offset[2];

    /* apply mounting matrix if configured */
    if (dev->config.mounting_matrix[0] != 0.0f ||
        dev->config.mounting_matrix[8] != 0.0f) {
        memcpy(temp_acc, data->acc, sizeof(temp_acc));
        memcpy(temp_gyro, data->gyro, sizeof(temp_gyro));

        mat3_mult_vec3(dev->config.mounting_matrix, temp_acc, data->acc);
        mat3_mult_vec3(dev->config.mounting_matrix, temp_gyro, data->gyro);
    }
}

int imu_init(struct imu_dev *dev, const struct imu_config *cfg)
{
    if (!dev || !dev->ops || !dev->ops->init)
        return -1;

    if (cfg) {
        dev->config = *cfg;
    } else {
        /* default config: identity matrix, 100Hz */
        memset(&dev->config, 0, sizeof(struct imu_config));
        dev->config.sample_rate = 100;
        dev->config.mounting_matrix[0] = 1;
        dev->config.mounting_matrix[4] = 1;
        dev->config.mounting_matrix[8] = 1;
    }

    return dev->ops->init(dev);
}

int imu_read(struct imu_dev *dev, struct imu_data *data)
{
    int ret;

    if (!dev || !dev->ops || !dev->ops->read)
        return -1;

    ret = dev->ops->read(dev, data);
    if (ret == 0)
        imu_apply_rotation_and_offset(dev, data);

    return ret;
}

void imu_set_callback(struct imu_dev *dev, imu_callback_t cb, void *ctx)
{
    if (dev) {
        dev->cb = cb;
        dev->cb_ctx = ctx;
    }
}

int imu_calibrate_gyro_bias(struct imu_dev *dev, uint32_t duration_ms)
{
    const int delay_us = 5000;
    int samples;
    double sum_gyro[3] = {0};
    struct imu_data data;
    int valid_count = 0;
    int i;

    if (!dev)
        return -1;

    samples = (duration_ms * 1000) / delay_us;
    if (samples <= 0)
        samples = 1;

    printf("IMU calibrating, keep still for %d ms\n", duration_ms);

    memset(dev->config.gyro_offset, 0, sizeof(dev->config.gyro_offset));

    for (i = 0; i < samples; i++) {
        if (imu_read(dev, &data) == 0) {
            sum_gyro[0] += data.gyro[0];
            sum_gyro[1] += data.gyro[1];
            sum_gyro[2] += data.gyro[2];
            valid_count++;
        }
        usleep(delay_us);
    }

    if (valid_count > 0) {
        dev->config.gyro_offset[0] = (float)(sum_gyro[0] / valid_count);
        dev->config.gyro_offset[1] = (float)(sum_gyro[1] / valid_count);
        dev->config.gyro_offset[2] = (float)(sum_gyro[2] / valid_count);
        printf("calibration done, offsets: %.4f, %.4f, %.4f\n",
                dev->config.gyro_offset[0], dev->config.gyro_offset[1],
                dev->config.gyro_offset[2]);
        return 0;
    }

    return -1;
}

void imu_free(struct imu_dev *dev)
{
    if (!dev)
        return;

    if (dev->ops && dev->ops->free) {
        dev->ops->free(dev);
        return;
    }

    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free((void *)dev->name);
    free(dev);
}

struct imu_dev *imu_dev_alloc(const char *name, size_t priv_size)
{
    struct imu_dev *dev;
    void *priv = NULL;
    char *name_copy = NULL;

    dev = calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;

    if (priv_size) {
        priv = calloc(1, priv_size);
        if (!priv) {
            free(dev);
            return NULL;
        }
        dev->priv_data = priv;
    }

    if (name) {
        size_t n = strlen(name);
        name_copy = calloc(1, n + 1);
        if (!name_copy) {
            free(priv);
            free(dev);
            return NULL;
        }
        memcpy(name_copy, name, n);
        name_copy[n] = '\0';
        dev->name = name_copy;
    }

    return dev;
}

/* --- driver registry (minimal, motor-like) --- */

static struct driver_info *g_driver_list = NULL;

void imu_driver_register(struct driver_info *info)
{
    if (!info)
        return;
    info->next = g_driver_list;
    g_driver_list = info;
}

static struct driver_info *find_driver(const char *name, enum imu_driver_type type)
{
    struct driver_info *curr = g_driver_list;
    while (curr) {
        if (curr->name && name && strcmp(curr->name, name) == 0) {
            if (curr->type == type)
                return curr;
            printf("[IMU] driver '%s' type mismatch (expected %d got %d)\n",
                    name, (int)type, (int)curr->type);
            return NULL;
        }
        curr = curr->next;
    }
    printf("[IMU] driver '%s' not found\n", name ? name : "(null)");
    return NULL;
}

static int split_driver_instance(const char *name, char *driver,
        size_t driver_sz, const char **instance)
{
    const char *sep;
    size_t len;

    if (!name || !driver || !driver_sz || !instance)
        return -1;

    sep = strchr(name, ':');
    if (!sep)
        return 0;

    len = (size_t)(sep - name);
    if (len == 0 || len + 1 > driver_sz || !*(sep + 1))
        return -1;

    memcpy(driver, name, len);
    driver[len] = '\0';
    *instance = sep + 1;
    return 1;
}

/* --- factory functions (public API) --- */

struct imu_dev *imu_alloc_i2c(const char *name, const char *i2c_dev, uint8_t addr,
        void *ex_args)
{
    struct driver_info *drv;
    struct imu_args_i2c args;
    char driver[32];
    const char *instance = NULL;
    int r;

    if (!name || !i2c_dev)
        return NULL;

    r = split_driver_instance(name, driver, sizeof(driver), &instance);
    if (r < 0)
        return NULL;
    if (r == 0) {
        strncpy(driver, name, sizeof(driver) - 1);
        driver[sizeof(driver) - 1] = '\0';
        instance = name;
    }

    drv = find_driver(driver, IMU_DRV_I2C);
    if (!drv || !drv->factory)
        return NULL;

    args.instance = instance;
    args.dev_path = i2c_dev;
    args.addr = addr;
    args.ex_args = ex_args;
    return drv->factory(&args);
}

struct imu_dev *imu_alloc_spi(const char *name, const char *spi_dev, uint32_t cs_pin,
        void *ex_args)
{
    struct driver_info *drv;
    struct imu_args_spi args;
    char driver[32];
    const char *instance = NULL;
    int r;

    if (!name || !spi_dev)
        return NULL;

    r = split_driver_instance(name, driver, sizeof(driver), &instance);
    if (r < 0)
        return NULL;
    if (r == 0) {
        strncpy(driver, name, sizeof(driver) - 1);
        driver[sizeof(driver) - 1] = '\0';
        instance = name;
    }

    drv = find_driver(driver, IMU_DRV_SPI);
    if (!drv || !drv->factory)
        return NULL;

    args.instance = instance;
    args.dev_path = spi_dev;
    args.cs_pin = cs_pin;
    args.ex_args = ex_args;
    return drv->factory(&args);
}

struct imu_dev *imu_alloc_uart(const char *name, const char *uart_dev, uint32_t baud,
        void *ex_args)
{
    struct driver_info *drv;
    struct imu_args_uart args;
    char driver[32];
    const char *instance = NULL;
    int r;

    if (!name || !uart_dev)
        return NULL;

    r = split_driver_instance(name, driver, sizeof(driver), &instance);
    if (r < 0)
        return NULL;
    if (r == 0) {
        strncpy(driver, name, sizeof(driver) - 1);
        driver[sizeof(driver) - 1] = '\0';
        instance = name;
    }

    drv = find_driver(driver, IMU_DRV_UART);
    if (!drv || !drv->factory)
        return NULL;

    args.instance = instance;
    args.dev_path = uart_dev;
    args.baud = baud;
    args.ex_args = ex_args;
    return drv->factory(&args);
}
