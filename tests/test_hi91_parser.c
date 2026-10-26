/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_hi91_parser.c
 * @brief Offline tests for partial/noisy HI91 packets, units, CRC and clock resets.
 */
#define _GNU_SOURCE
#include "hi91_parser.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static void write_le(uint8_t *bytes, uint32_t value) {
    for (size_t i = 0; i < 4; ++i) bytes[i] = (uint8_t)(value >> (8 * i));
}

static void write_float(uint8_t *bytes, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    write_le(bytes, bits);
}

static void checksum(uint8_t *bytes) {
    uint16_t crc = 0;
    for (size_t i = 0; i < HI91_FRAME_SIZE; ++i) {
        if (i == 4 || i == 5) continue;
        crc ^= (uint16_t)bytes[i] << 8;
        for (size_t bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0));
    }
    bytes[4] = (uint8_t)crc;
    bytes[5] = (uint8_t)(crc >> 8);
}

static void frame(uint8_t *bytes, uint32_t tick) {
    memset(bytes, 0, HI91_FRAME_SIZE);
    bytes[0] = 0x5a;
    bytes[1] = 0xa5;
    bytes[2] = 76;
    bytes[6] = 0x91;
    write_le(bytes + 14, tick);
    write_float(bytes + 18, 1.0f);
    write_float(bytes + 30, 180.0f);
    write_float(bytes + 66, 1.0f);
    checksum(bytes);
}

int main(void) {
    struct hi91_parser parser;
    struct imu_data sample;
    uint8_t bytes[HI91_FRAME_SIZE];
    hi91_parser_init(&parser);
    frame(bytes, UINT32_MAX - 1);
    assert(hi91_parser_feed(&parser, (const uint8_t *)"junk", 4, 1, &sample) == -EAGAIN);
    assert(hi91_parser_feed(&parser, bytes, 5, 2, &sample) == -EAGAIN);
    assert(hi91_parser_feed(&parser, bytes + 5, sizeof(bytes) - 5, 3, &sample) == 0);
    assert(fabs(sample.acc[0] - 9.80665) < 1.0e-5);
    assert(fabs(sample.gyro[0] - 3.14159265) < 1.0e-5 && sample.quat[0] == 1.0f);
    assert(parser.diagnostics.receive_timestamp_us == 3);
    frame(bytes, 1);  // Natural uint32 wrap, not a clock reset.
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 4, &sample) == 0);
    assert(sample.timestamp_us == (UINT64_C(1) << 32) * 1000 + 1000);
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 5, &sample) == -EAGAIN);
    assert(parser.diagnostics.receive_timestamp_us == 4);
    bytes[30] ^= 1;
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 6, &sample) == -EAGAIN);
    assert(parser.diagnostics.crc_errors == 1);
    frame(bytes, 2);
    write_float(bytes + 66, NAN);
    checksum(bytes);
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 7, &sample) == -EAGAIN);
    frame(bytes, 0);
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 8, &sample) == -EIO);
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 9, &sample) == -EIO);
    hi91_parser_init(&parser);
    frame(bytes, 3);
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 1, &sample) == 0);
    assert(hi91_parser_feed(&parser, bytes, sizeof(bytes), 0, &sample) == -EINVAL);

    int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    assert(master >= 0 && grantpt(master) == 0 && unlockpt(master) == 0);
    struct imu_dev *dev = imu_alloc_uart("drv_uart_hi91:pty", ptsname(master), 921600, NULL);
    struct imu_config config = {.mounting_matrix = {1, 0, 0, 0, 1, 0, 0, 0, 1}};
    struct imu_diagnostics diagnostics;
    assert(dev != NULL && imu_init(dev, &config) == 0);
    assert(write(master, bytes, sizeof(bytes)) == sizeof(bytes));
    // PTY delivery is asynchronous; a bounded poll retries without real devices.
    int result = -EAGAIN;
    for (size_t attempt = 0; attempt < 100 && result == -EAGAIN; ++attempt) {
        result = imu_read(dev, &sample);
        if (result == -EAGAIN) usleep(1000);
    }
    assert(result == 0 && sample.timestamp_us == 3000);
    assert(imu_get_diagnostics(dev, &diagnostics) == 0 && diagnostics.valid_frames == 1);
    assert(imu_read(dev, &sample) == -EAGAIN);
    imu_free(dev);
    close(master);
    return 0;
}
