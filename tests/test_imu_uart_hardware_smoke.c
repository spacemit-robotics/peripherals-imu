/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/imu.h"

static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s <device> <baud> <rate_hz> <samples>\n", prog);
}

int main(int argc, char **argv)
{
    const char *dev_path;
    uint32_t baud;
    uint32_t rate_hz;
    uint32_t samples;
    struct imu_dev *imu;
    struct imu_config cfg = {0};
    struct imu_data data;
    uint32_t count;

    if (argc != 5) {
        usage(argv[0]);
        return 1;
    }

    dev_path = argv[1];
    baud = (uint32_t)strtoul(argv[2], NULL, 10);
    rate_hz = (uint32_t)strtoul(argv[3], NULL, 10);
    samples = (uint32_t)strtoul(argv[4], NULL, 10);

    if (!dev_path[0] || baud == 0U || rate_hz == 0U || samples == 0U) {
        usage(argv[0]);
        return 1;
    }

    cfg.sample_rate = rate_hz;
    cfg.mounting_matrix[0] = 1.0f;
    cfg.mounting_matrix[4] = 1.0f;
    cfg.mounting_matrix[8] = 1.0f;

    imu = imu_alloc_uart("CMP10A:cmp10a_imu", dev_path, baud, NULL);
    if (!imu) {
        fprintf(stderr, "Failed to allocate IMU device\n");
        return 1;
    }

    if (imu_init(imu, &cfg) != 0) {
        fprintf(stderr, "Failed to initialize IMU\n");
        imu_free(imu);
        return 1;
    }

    printf("IMU initialized successfully\n");
    printf("device=%s baud=%u rate=%uHz samples=%u\n",
            dev_path, baud, rate_hz, samples);
    printf("%-12s | %-30s | %-30s | %-8s\n",
            "Timestamp", "Accel (m/s^2)", "Gyro (rad/s)", "Temp");

    for (count = 0; count < samples; ++count) {
        if (imu_read(imu, &data) != 0) {
            fprintf(stderr, "Read error at sample %u\n", count + 1U);
            imu_free(imu);
            return 1;
        }

        printf("%12" PRIu64 " | %9.4f %9.4f %9.4f | %9.5f %9.5f %9.5f | %8.2f\n",
                data.timestamp_us,
                data.acc[0], data.acc[1], data.acc[2],
                data.gyro[0], data.gyro[1], data.gyro[2],
                data.temp);
    }

    printf("Total samples: %u\n", samples);
    imu_free(imu);
    return 0;
}
