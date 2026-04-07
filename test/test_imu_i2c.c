/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "../include/imu.h"

static volatile int g_running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
    printf("\nStopping...\n");
}

static void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("Options:\n");
    printf("  -d <device>   IIO device selector, e.g. iio:device0 or 0\n");
    printf("  -i <index>    IIO device index, used when -d is empty (default: 0)\n");
    printf("  -r <rate>     Print rate in Hz (default: 10)\n");
    printf("  -n <count>    Number of samples (default: 0, infinite)\n");
    printf("  -h            Show this help\n");
}

int main(int argc, char *argv[])
{
    const char *device_selector = "";
    uint8_t device_index = 0;
    int print_rate = 10;
    int sample_count = 0;
    int count = 0;
    int opt;
    struct imu_dev *imu;
    struct imu_config cfg = {0};
    struct imu_data data;

    while ((opt = getopt(argc, argv, "d:i:r:n:h")) != -1) {
        switch (opt) {
        case 'd':
            device_selector = optarg;
            break;
        case 'i': {
            uint32_t value = (uint32_t)strtoul(optarg, NULL, 10);
            if (value > 255U)
                value = 255U;
            device_index = (uint8_t)value;
            break;
        }
        case 'r':
            print_rate = atoi(optarg);
            if (print_rate <= 0)
                print_rate = 10;
            break;
        case 'n':
            sample_count = atoi(optarg);
            break;
        case 'h':
        default:
            print_usage(argv[0]);
            return (opt == 'h') ? 0 : -1;
        }
    }

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    cfg.sample_rate = (uint32_t)print_rate;
    cfg.mounting_matrix[0] = 1.0f;
    cfg.mounting_matrix[4] = 1.0f;
    cfg.mounting_matrix[8] = 1.0f;

    imu = imu_alloc_i2c("mxc4005",
            device_selector, device_index, NULL);
    if (!imu) {
        fprintf(stderr, "Failed to allocate I2C IMU\n");
        return -1;
    }

    if (imu_init(imu, &cfg) != 0) {
        fprintf(stderr, "Failed to initialize I2C IMU\n");
        imu_free(imu);
        return -1;
    }

    printf("=== IMU I2C Test ===\n");
    printf("selector=%s index=%u rate=%dHz\n",
            device_selector[0] ? device_selector : "(empty)",
            (unsigned int)device_index, print_rate);
    printf("%-14s %-12s %-12s %-12s\n", "timestamp_us", "acc_x_raw",
            "acc_y_raw", "acc_z_raw");

    while (g_running) {
        if (imu_read(imu, &data) == 0) {
            printf("%-14" PRIu64 " %-12.0f %-12.0f %-12.0f\n",
                    data.timestamp_us, data.acc[0], data.acc[1], data.acc[2]);
            count++;
            if (sample_count > 0 && count >= sample_count)
                break;
        } else {
            printf("read failed\n");
        }

        usleep((useconds_t)(1000000 / print_rate));
    }

    imu_free(imu);
    return 0;
}
