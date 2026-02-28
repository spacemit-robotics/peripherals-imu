/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * IMU UART Test
 * Test program for IMU sensor via UART interface
 */

#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    printf("  -d <device>   Serial device path (default: /dev/ttyUSB0)\n");
    printf("  -b <baud>     Baud rate (default: 9600)\n");
    printf("  -r <rate>     Print rate in Hz (default: 10)\n");
    printf("  -c <ms>       Calibration duration in ms (default: 0, disabled)\n");
    printf("  -n <count>    Number of samples (default: 0, infinite)\n");
    printf("  -h            Show this help\n");
}

int main(int argc, char *argv[])
{
    const char *dev_path = "/dev/ttyUSB0";
    uint32_t baud = 9600;
    int print_rate = 10;
    int calib_ms = 0;
    int sample_count = 0;
    int opt;

    /* parse arguments */
    while ((opt = getopt(argc, argv, "d:b:r:c:n:h")) != -1) {
        switch (opt) {
        case 'd':
            dev_path = optarg;
            break;
        case 'b':
            baud = (uint32_t)atoi(optarg);
            break;
        case 'r':
            print_rate = atoi(optarg);
            if (print_rate <= 0) print_rate = 10;
            break;
        case 'c':
            calib_ms = atoi(optarg);
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

    printf("=== CMP10A IMU UART Test ===\n");
    printf("Device: %s, Baud: %u, Rate: %d Hz\n", dev_path, baud, print_rate);

    /* setup signal handler */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 1. create instance */
    struct imu_dev *imu = imu_alloc_uart("CMP10A:cmp10a_imu", dev_path, baud, NULL);
    if (!imu) {
        printf("Failed to allocate IMU device\n");
        return -1;
    }

    /* 2. prepare config */
    struct imu_config cfg = {0};
    cfg.sample_rate = 50;  /* CMP10A typically outputs at 50Hz */

    /* identity mounting matrix */
    cfg.mounting_matrix[0] = 1; cfg.mounting_matrix[1] = 0; cfg.mounting_matrix[2] = 0;
    cfg.mounting_matrix[3] = 0; cfg.mounting_matrix[4] = 1; cfg.mounting_matrix[5] = 0;
    cfg.mounting_matrix[6] = 0; cfg.mounting_matrix[7] = 0; cfg.mounting_matrix[8] = 1;

    /* 3. initialize */
    if (imu_init(imu, &cfg) != 0) {
        printf("Failed to initialize IMU\n");
        imu_free(imu);
        return -1;
    }

    printf("IMU initialized successfully\n");

    /* 4. optional calibration */
    if (calib_ms > 0) {
        printf("Starting gyro calibration for %d ms...\n", calib_ms);
        if (imu_calibrate_gyro_bias(imu, (uint32_t)calib_ms) != 0) {
            printf("Calibration failed\n");
        }
    }

    /* 5. read loop */
    struct imu_data data;
    int count = 0;
    int sleep_us = 1000000 / print_rate;

    printf("\nReading IMU data (Ctrl+C to stop)...\n");
    printf("%-12s | %-30s | %-30s | %-40s\n",
            "Timestamp", "Accel (m/s^2)", "Gyro (rad/s)", "Quaternion (w,x,y,z)");
    printf("-------------+--------------------------------+--------------------------------+"
            "-----------------------------------------\n");

    while (g_running) {
        int ret = imu_read(imu, &data);
        if (ret == 0) {
            printf("%12" PRIu64 " | %9.4f %9.4f %9.4f | %9.5f %9.5f %9.5f | "
                    "%8.4f %8.4f %8.4f %8.4f\n",
                    data.timestamp_us,
                    data.acc[0], data.acc[1], data.acc[2],
                    data.gyro[0], data.gyro[1], data.gyro[2],
                    data.quat[0], data.quat[1], data.quat[2], data.quat[3]);

            count++;
            if (sample_count > 0 && count >= sample_count) {
                break;
            }
        } else {
            printf("Read error\n");
        }

        usleep(sleep_us);
    }

    printf("\nTotal samples: %d\n", count);

    /* 6. cleanup */
    imu_free(imu);
    printf("IMU released\n");

    return 0;
}
