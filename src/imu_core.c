/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file imu_core.c
 * @brief IMU registry, mounting transform, and calibration implementation.
 */
#include "imu_core.h"
#include <math.h>
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

static void mat3_to_quat(const float R[9], float quat[4])
{
    float scale;
    float trace = R[0] + R[4] + R[8];

    if (trace > 0.0f) {
        scale = 2.0f * sqrtf(trace + 1.0f);
        quat[0] = 0.25f * scale;
        quat[1] = (R[7] - R[5]) / scale;
        quat[2] = (R[2] - R[6]) / scale;
        quat[3] = (R[3] - R[1]) / scale;
    } else if (R[0] > R[4] && R[0] > R[8]) {
        scale = 2.0f * sqrtf(1.0f + R[0] - R[4] - R[8]);
        quat[0] = (R[7] - R[5]) / scale;
        quat[1] = 0.25f * scale;
        quat[2] = (R[1] + R[3]) / scale;
        quat[3] = (R[2] + R[6]) / scale;
    } else if (R[4] > R[8]) {
        scale = 2.0f * sqrtf(1.0f + R[4] - R[0] - R[8]);
        quat[0] = (R[2] - R[6]) / scale;
        quat[1] = (R[1] + R[3]) / scale;
        quat[2] = 0.25f * scale;
        quat[3] = (R[5] + R[7]) / scale;
    } else {
        scale = 2.0f * sqrtf(1.0f + R[8] - R[0] - R[4]);
        quat[0] = (R[3] - R[1]) / scale;
        quat[1] = (R[2] + R[6]) / scale;
        quat[2] = (R[5] + R[7]) / scale;
        quat[3] = 0.25f * scale;
    }
}

static void quat_multiply(const float lhs[4], const float rhs[4], float out[4])
{
    out[0] = lhs[0] * rhs[0] - lhs[1] * rhs[1] -
        lhs[2] * rhs[2] - lhs[3] * rhs[3];
    out[1] = lhs[0] * rhs[1] + lhs[1] * rhs[0] +
        lhs[2] * rhs[3] - lhs[3] * rhs[2];
    out[2] = lhs[0] * rhs[2] - lhs[1] * rhs[3] +
        lhs[2] * rhs[0] + lhs[3] * rhs[1];
    out[3] = lhs[0] * rhs[3] + lhs[1] * rhs[2] -
        lhs[2] * rhs[1] + lhs[3] * rhs[0];
}

static void quat_conjugate(const float in[4], float out[4])
{
    out[0] = in[0];
    out[1] = -in[1];
    out[2] = -in[2];
    out[3] = -in[3];
}

static int quat_normalize(float quat[4])
{
    float norm = sqrtf(quat[0] * quat[0] + quat[1] * quat[1] +
        quat[2] * quat[2] + quat[3] * quat[3]);

    if (!isfinite(norm) || norm <= 1.0e-8f)
        return -1;
    quat[0] /= norm;
    quat[1] /= norm;
    quat[2] /= norm;
    quat[3] /= norm;
    return 0;
}

static int quat_is_available(const float quat[4])
{
    return quat[0] != 0.0f || quat[1] != 0.0f ||
        quat[2] != 0.0f || quat[3] != 0.0f;
}

static int matrix_is_configured(const float matrix[9])
{
    int i;

    for (i = 0; i < 9; ++i) {
        if (matrix[i] != 0.0f)
            return 1;
    }
    return 0;
}

void imu_apply_rotation_and_offset(struct imu_dev *dev, struct imu_data *data)
{
    float temp_acc[3], temp_gyro[3], temp_mag[3];
    int has_quaternion = quat_is_available(data->quat);

    /* subtract bias offset */
    data->acc[0] -= dev->config.acc_offset[0];
    data->acc[1] -= dev->config.acc_offset[1];
    data->acc[2] -= dev->config.acc_offset[2];

    data->gyro[0] -= dev->config.gyro_offset[0];
    data->gyro[1] -= dev->config.gyro_offset[1];
    data->gyro[2] -= dev->config.gyro_offset[2];

    /* apply mounting matrix if configured */
    if (matrix_is_configured(dev->config.mounting_matrix)) {
        memcpy(temp_acc, data->acc, sizeof(temp_acc));
        memcpy(temp_gyro, data->gyro, sizeof(temp_gyro));
        memcpy(temp_mag, data->mag, sizeof(temp_mag));

        mat3_mult_vec3(dev->config.mounting_matrix, temp_acc, data->acc);
        mat3_mult_vec3(dev->config.mounting_matrix, temp_gyro, data->gyro);
        mat3_mult_vec3(dev->config.mounting_matrix, temp_mag, data->mag);

        if (has_quaternion) {
            float sensor_to_body_quat[4];
            float body_to_sensor_quat[4];
            float sensor_quat[4];

            memcpy(sensor_quat, data->quat, sizeof(sensor_quat));
            mat3_to_quat(dev->config.mounting_matrix,
                    sensor_to_body_quat);
            quat_normalize(sensor_to_body_quat);
            quat_conjugate(sensor_to_body_quat, body_to_sensor_quat);
            quat_multiply(sensor_quat, body_to_sensor_quat, data->quat);
            quat_normalize(data->quat);
        }
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

    if (!dev || !dev->ops || !dev->ops->read)
        return -1;

    samples = (duration_ms * 1000) / delay_us;
    if (samples <= 0)
        samples = 1;

    printf("IMU calibrating, keep still for %d ms\n", duration_ms);

    memset(dev->config.gyro_offset, 0, sizeof(dev->config.gyro_offset));

    for (i = 0; i < samples; i++) {
        /* Offsets are stored in the sensor frame and rotated during imu_read. */
        if (dev->ops->read(dev, &data) == 0) {
            sum_gyro[0] += data.gyro[0];
            sum_gyro[1] += data.gyro[1];
            sum_gyro[2] += data.gyro[2];
            valid_count++;
        }
        usleep(delay_us);
    }

    if (valid_count > 0) {
        dev->config.gyro_offset[0] = sum_gyro[0] / valid_count;
        dev->config.gyro_offset[1] = sum_gyro[1] / valid_count;
        dev->config.gyro_offset[2] = sum_gyro[2] / valid_count;
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
        free(dev->name);
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

static void dump_registered_drivers(void)
{
    struct driver_info *curr = g_driver_list;

    printf("[IMU] registered drivers:");
    if (!curr) {
        printf(" (none)\n");
        return;
    }

    while (curr) {
        printf(" %s(type=%d)", curr->name ? curr->name : "(null)",
                curr->type);
        curr = curr->next;
        if (curr)
            printf(",");
    }
    printf("\n");
}

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
                    name, type, curr->type);
            return NULL;
        }
        curr = curr->next;
    }
    printf("[IMU] driver '%s' not found\n", name ? name : "(null)");
    dump_registered_drivers();
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

    len = sep - name;
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
