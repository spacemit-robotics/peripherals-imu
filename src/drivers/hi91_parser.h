/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file hi91_parser.h
 * @brief Driver-private bounded streaming decoder for HI91 sensor frames.
 */
#ifndef HI91_PARSER_H
#define HI91_PARSER_H

#include <stddef.h>
#include <stdint.h>

#include "imu.h"

#define HI91_FRAME_SIZE 82U

struct hi91_parser {
    uint8_t bytes[HI91_FRAME_SIZE];
    size_t used;
    uint32_t last_tick_ms;
    uint64_t extended_tick_ms;
    uint64_t last_host_us;
    int has_tick;
    int clock_fault;
    struct imu_diagnostics diagnostics;
};

void hi91_parser_init(struct hi91_parser *parser);
/* 0: newest fresh sample; -EAGAIN: no valid sample; -EIO: sensor clock reset.
 * Backward sensor ticks latch a fault until explicit parser reinitialization. */
int hi91_parser_feed(struct hi91_parser *parser, const uint8_t *bytes, size_t size,
    uint64_t received_us, struct imu_data *sample);

#endif  // HI91_PARSER_H
