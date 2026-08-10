/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_imu_uart.c
 * @brief Manual UART IMU test for any enabled UART driver.
 */

#include <errno.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "imu.h"

#define RAD_TO_DEG (180.0f / 3.14159265358979323846f)

static volatile int g_running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    g_running = 0;
    printf("\nStopping...\n");
}

static void print_usage(const char *program)
{
    printf("Usage: %s [options]\n", program);
    printf("Options:\n");
    printf("  -t <driver>   Driver[:instance] (default: CMP10A:cmp10a_imu)\n");
    printf("  -d <device>   Serial device path (default: /dev/ttyUSB0)\n");
    printf("  -b <baud>     Baud rate (default: 9600)\n");
    printf("  -r <rate>     Read/print rate in Hz (default: 10)\n");
    printf("  -m <matrix>   Row-major sensor-to-body matrix, 9 comma-separated floats\n");
    printf("  -c <ms>       Gyro calibration duration in ms (default: disabled)\n");
    printf("  -n <count>    Number of samples (default: infinite)\n");
    printf("  -h            Show this help\n");
}

static int parse_mounting_matrix(const char *text, float matrix[9])
{
    const char *cursor = text;
    int i;

    if (!text)
        return -1;
    for (i = 0; i < 9; ++i) {
        char *end;

        errno = 0;
        matrix[i] = strtof(cursor, &end);
        if (errno != 0 || end == cursor)
            return -1;
        if (i < 8) {
            if (*end != ',')
                return -1;
            cursor = end + 1;
        } else if (*end != '\0') {
            return -1;
        }
    }
    return 0;
}

static void quat_to_rpy(const float quat[4], float rpy[3])
{
    float pitch_sine = 2.0f * (quat[0] * quat[2] - quat[3] * quat[1]);

    rpy[0] = atan2f(2.0f * (quat[0] * quat[1] + quat[2] * quat[3]),
            1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]));
    if (fabsf(pitch_sine) >= 1.0f)
        rpy[1] = copysignf(1.57079632679489661923f, pitch_sine);
    else
        rpy[1] = asinf(pitch_sine);
    rpy[2] = atan2f(2.0f * (quat[0] * quat[3] + quat[1] * quat[2]),
            1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]));
}

int main(int argc, char *argv[])
{
    const char *driver = "CMP10A:cmp10a_imu";
    const char *dev_path = "/dev/ttyUSB0";
    const char *matrix_text = NULL;
    struct imu_config config = {0};
    struct imu_dev *imu;
    struct imu_data data;
    uint32_t baud = 9600U;
    int print_rate = 10;
    int calibration_ms = 0;
    int sample_count = 0;
    int count = 0;
    int missed_reads = 0;
    int stream_failed = 0;
    int opt;

    config.mounting_matrix[0] = 1.0f;
    config.mounting_matrix[4] = 1.0f;
    config.mounting_matrix[8] = 1.0f;

    while ((opt = getopt(argc, argv, "t:d:b:r:m:c:n:h")) != -1) {
        switch (opt) {
        case 't':
            driver = optarg;
            break;
        case 'd':
            dev_path = optarg;
            break;
        case 'b':
            baud = (uint32_t)strtoul(optarg, NULL, 10);
            break;
        case 'r':
            print_rate = atoi(optarg);
            break;
        case 'm':
            matrix_text = optarg;
            break;
        case 'c':
            calibration_ms = atoi(optarg);
            break;
        case 'n':
            sample_count = atoi(optarg);
            break;
        case 'h':
        default:
            print_usage(argv[0]);
            return opt == 'h' ? 0 : -1;
        }
    }
    if (print_rate <= 0 || baud == 0U) {
        print_usage(argv[0]);
        return -1;
    }
    if (matrix_text && parse_mounting_matrix(
        matrix_text, config.mounting_matrix) < 0) {
        fprintf(stderr, "Invalid mounting matrix\n");
        return -1;
    }
    config.sample_rate = (uint32_t)print_rate;

    printf("=== IMU UART Test ===\n");
    printf("Driver: %s, device: %s, baud: %u, rate: %d Hz\n",
            driver, dev_path, baud, print_rate);
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    imu = imu_alloc_uart(driver, dev_path, baud, NULL);
    if (!imu) {
        fprintf(stderr, "Failed to allocate IMU device\n");
        return -1;
    }
    if (imu_init(imu, &config) != 0) {
        fprintf(stderr, "Failed to initialize IMU\n");
        imu_free(imu);
        return -1;
    }
    printf("IMU initialized successfully\n");

    if (calibration_ms > 0 &&
        imu_calibrate_gyro_bias(imu, (uint32_t)calibration_ms) != 0) {
        fprintf(stderr, "Calibration failed\n");
    }

    printf("%-12s | %-25s | %-25s | %-22s | %s\n",
            "Timestamp", "Accel (m/s^2)", "Gyro (rad/s)", "RPY (deg)", "Temp (C)");
    while (g_running) {
        int result = imu_read(imu, &data);

        if (result == 0) {
            float rpy[3];

            quat_to_rpy(data.quat, rpy);
            printf("%12" PRIu64 " | %8.3f %8.3f %8.3f | %8.4f %8.4f %8.4f | "
                    "%7.2f %7.2f %7.2f | %6.1f\n",
                    data.timestamp_us, data.acc[0], data.acc[1], data.acc[2],
                    data.gyro[0], data.gyro[1], data.gyro[2],
                    rpy[0] * RAD_TO_DEG, rpy[1] * RAD_TO_DEG,
                    rpy[2] * RAD_TO_DEG, data.temp);
            ++count;
            missed_reads = 0;
            if (sample_count > 0 && count >= sample_count)
                break;
        } else if (++missed_reads >= print_rate * 2) {
            fprintf(stderr, "No valid IMU frame received for 2 seconds\n");
            stream_failed = 1;
            break;
        }
        usleep(1000000U / (uint32_t)print_rate);
    }

    printf("Total samples: %d\n", count);
    imu_free(imu);
    return count > 0 && !stream_failed ? 0 : -1;
}
