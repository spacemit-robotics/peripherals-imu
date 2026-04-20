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
    printf("  -d <device>   SPI device path (default: /dev/spidev0.0)\n");
    printf("  -s <speed>    SPI clock in Hz (default: 1000000)\n");
    printf("  -m <mode>     SPI mode 0..3 (default: 2)\n");
    printf("  -r <rate>     Sensor ODR and print rate in Hz (default: 100)\n");
    printf("  -l <freq>     DLPF target frequency in Hz (default: 0, keep reset default)\n");
    printf("  -c <ms>       Calibration duration in ms (default: 0)\n");
    printf("  -n <count>    Number of samples (default: 0, infinite)\n");
    printf("  -p <cs_pin>   Logical CS pin index for bookkeeping (default: 0)\n");
    printf("  -h            Show this help\n");
}

int main(int argc, char *argv[])
{
    const char *dev_path = "/dev/spidev0.0";
    struct imu_spi_config spi_cfg = {
        .mode = 2U,
        .bits_per_word = 8U,
        .speed_hz = 1000000U,
    };
    struct imu_config cfg = {0};
    struct imu_dev *imu;
    struct imu_data data;
    uint32_t cs_pin = 0U;
    int print_rate = 100;
    int calib_ms = 0;
    int sample_count = 0;
    int count = 0;
    int opt;

    while ((opt = getopt(argc, argv, "d:s:m:r:l:c:n:p:h")) != -1) {
        switch (opt) {
        case 'd':
            dev_path = optarg;
            break;
        case 's':
            spi_cfg.speed_hz = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 'm': {
            uint32_t mode = (uint32_t)strtoul(optarg, NULL, 10);

            if (mode > 3U)
                mode = 3U;
            spi_cfg.mode = (uint8_t)mode;
            break;
        }
        case 'r':
            print_rate = atoi(optarg);
            if (print_rate <= 0)
                print_rate = 100;
            break;
        case 'l':
            cfg.dlpf_freq = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 'c':
            calib_ms = atoi(optarg);
            if (calib_ms < 0)
                calib_ms = 0;
            break;
        case 'n':
            sample_count = atoi(optarg);
            break;
        case 'p':
            cs_pin = (uint32_t)strtoul(optarg, NULL, 10);
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

    imu = imu_alloc_spi("icm42670p", dev_path, cs_pin, &spi_cfg);
    if (!imu) {
        fprintf(stderr, "Failed to allocate SPI IMU\n");
        return -1;
    }

    if (imu_init(imu, &cfg) != 0) {
        fprintf(stderr, "Failed to initialize SPI IMU\n");
        imu_free(imu);
        return -1;
    }

    if (calib_ms > 0) {
        if (imu_calibrate_gyro_bias(imu, (uint32_t)calib_ms) != 0) {
            fprintf(stderr, "Gyro calibration failed\n");
        }
    }

    printf("=== ICM-42670-P SPI Test ===\n");
    printf("device=%s speed=%u mode=%u rate=%dHz dlpf=%uHz cs=%u\n",
            dev_path, spi_cfg.speed_hz, spi_cfg.mode, print_rate,
            cfg.dlpf_freq, cs_pin);
    printf("%-14s %-26s %-26s %-8s\n",
            "timestamp_us", "acc(m/s^2)", "gyro(rad/s)", "temp(C)");

    while (g_running) {
        if (imu_read(imu, &data) == 0) {
            printf("%-14" PRIu64 " %8.3f %8.3f %8.3f   %8.4f %8.4f %8.4f   %8.2f\n",
                    data.timestamp_us,
                    data.acc[0], data.acc[1], data.acc[2],
                    data.gyro[0], data.gyro[1], data.gyro[2],
                    data.temp);
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
