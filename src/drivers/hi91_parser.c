/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file hi91_parser.c
 * @brief Driver-private HI91 CRC, unit conversion and tick extension.
 */
#include "hi91_parser.h"

#include <errno.h>
#include <math.h>
#include <string.h>

static uint32_t read_le(const uint8_t *bytes, size_t size) {
    uint32_t value = 0;
    for (size_t i = 0; i < size; ++i)
        value |= (uint32_t)bytes[i] << (8 * i);
    return value;
}

static float read_float(const uint8_t *bytes) {
    uint32_t bits = read_le(bytes, 4);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static uint16_t crc16(uint16_t crc, const uint8_t *bytes, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (size_t bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000U) ? 0x1021U : 0));
    }
    return crc;
}

static int decode(struct hi91_parser *parser, struct imu_data *sample) {
    const uint8_t *payload = parser->bytes + 6;
    struct imu_data data = {0};
    float values[16];
    double norm = 0.0;
    uint32_t tick;
    uint32_t delta = 0;
    if (payload[0] != 0x91 || !isfinite(read_float(payload + 4)))
        return -EINVAL;
    for (size_t i = 0; i < 16; ++i) {
        values[i] = read_float(payload + 12 + i * 4);
        if (!isfinite(values[i])) return -EINVAL;
    }
    for (size_t i = 0; i < 4; ++i) norm += (double)values[12 + i] * values[12 + i];
    norm = sqrt(norm);
    if (fabs(norm - 1.0) > 0.01)
        return -EINVAL;
    for (size_t i = 0; i < 4; ++i) data.quat[i] = (float)(values[12 + i] / norm);
    for (size_t i = 0; i < 3; ++i) {
        data.acc[i] = values[i] * 9.80665f;
        data.gyro[i] = values[3 + i] * (3.14159265358979323846f / 180.0f);
        data.mag[i] = values[6 + i];
        if (!isfinite(data.acc[i]) || !isfinite(data.gyro[i])) return -EINVAL;
    }
    data.temp = payload[3] < 128 ? payload[3] : (int)payload[3] - 256;
    tick = read_le(payload + 8, 4);
    if (parser->has_tick) {
        delta = tick - parser->last_tick_ms;
        if (delta == 0) return -EAGAIN;
        if (delta >= UINT32_C(0x80000000)) {
            parser->clock_fault = 1;
            return -EIO;
        }
    }
    parser->extended_tick_ms = parser->has_tick ? parser->extended_tick_ms + delta : tick;
    parser->has_tick = 1;
    parser->last_tick_ms = tick;
    data.timestamp_us = parser->extended_tick_ms * 1000ULL;
    *sample = data;
    return 0;
}

static void discard(struct hi91_parser *parser) {
    --parser->used;
    memmove(parser->bytes, parser->bytes + 1, parser->used);
    ++parser->diagnostics.resync_discarded_bytes;
}

static int consume(struct hi91_parser *parser, struct imu_data *sample) {
    while (parser->used) {
        uint16_t crc;
        int result;
        if (parser->bytes[0] != 0x5a ||
            (parser->used >= 2 && parser->bytes[1] != 0xa5)) {
            discard(parser);
            continue;
        }
        if (parser->used < 4) return -EAGAIN;
        if (read_le(parser->bytes + 2, 2) != 76) {
            ++parser->diagnostics.decode_errors;
            discard(parser);
            continue;
        }
        if (parser->used < HI91_FRAME_SIZE) return -EAGAIN;
        crc = crc16(0, parser->bytes, 4);
        crc = crc16(crc, parser->bytes + 6, 76);
        if (crc != read_le(parser->bytes + 4, 2)) {
            ++parser->diagnostics.crc_errors;
            discard(parser);
            continue;
        }
        result = decode(parser, sample);
        parser->used = 0;
        if (result < 0) {
            ++parser->diagnostics.decode_errors;
            return result;
        }
        ++parser->diagnostics.valid_frames;
        return 0;
    }
    return -EAGAIN;
}

void hi91_parser_init(struct hi91_parser *parser) {
    if (parser) memset(parser, 0, sizeof(*parser));
}

int hi91_parser_feed(struct hi91_parser *parser, const uint8_t *bytes, size_t size,
    uint64_t received_us, struct imu_data *sample) {
    int found = 0;
    if (!parser || (!bytes && size > 0) || !sample || received_us == 0 ||
        received_us < parser->last_host_us)
        return -EINVAL;
    if (parser->clock_fault) return -EIO;
    parser->last_host_us = received_us;
    for (size_t i = 0; i < size; ++i) {
        int result;
        parser->bytes[parser->used++] = bytes[i];
        result = consume(parser, sample);
        if (result == -EIO) return -EIO;
        if (result == 0) {
            if (found) ++parser->diagnostics.superseded_frames;
            found = 1;
            parser->diagnostics.receive_timestamp_us = received_us;
        }
    }
    return found ? 0 : -EAGAIN;
}
