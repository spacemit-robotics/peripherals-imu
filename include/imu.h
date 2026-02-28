/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * struct imu_data - sensor data
 * @timestamp_us: sample timestamp (microseconds)
 * @acc:          accelerometer (m/s^2)
 * @gyro:         gyroscope (rad/s)
 * @mag:          magnetometer (optional, 0 if unavailable)
 * @quat:         quaternion w,x,y,z (optional, from DMP)
 * @temp:         temperature (Celsius)
 */
struct imu_data {
    uint64_t timestamp_us;
    float    acc[3];
    float    gyro[3];
    float    mag[3];
    float    quat[4];
    float    temp;
};

/*
 * struct imu_config - sensor configuration
 * @mounting_matrix: 3x3 rotation matrix (sensor to body frame)
 * @acc_offset:      accelerometer bias offset
 * @gyro_offset:     gyroscope bias offset
 * @sample_rate:     sample rate in Hz
 * @dlpf_freq:       low-pass filter cutoff frequency (Hz)
 */
struct imu_config {
    float    mounting_matrix[9];
    float    acc_offset[3];
    float    gyro_offset[3];
    uint32_t sample_rate;
    uint32_t dlpf_freq;
};

/* opaque handle */
struct imu_dev;

typedef void (*imu_callback_t)(struct imu_dev *dev,
        const struct imu_data *data, void *ctx);

/* --- core API --- */

int imu_init(struct imu_dev *dev, const struct imu_config *cfg);
int imu_read(struct imu_dev *dev, struct imu_data *data);
void imu_set_callback(struct imu_dev *dev, imu_callback_t cb, void *ctx);
int imu_calibrate_gyro_bias(struct imu_dev *dev, uint32_t duration_ms);
void imu_free(struct imu_dev *dev);

/* --- factory functions --- */

struct imu_dev *imu_alloc_i2c(const char *name, const char *i2c_dev,
        uint8_t addr, void *ex_args);
struct imu_dev *imu_alloc_spi(const char *name, const char *spi_dev,
        uint32_t cs_pin, void *ex_args);
struct imu_dev *imu_alloc_uart(const char *name, const char *uart_dev,
        uint32_t baud, void *ex_args);

#ifdef __cplusplus
}
#endif

#endif  /* IMU_H */
