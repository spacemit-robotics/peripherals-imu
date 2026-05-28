/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/imu_core.h"

struct fake_priv {
    char dev_path[128];
    uint32_t bus_arg;
    uint8_t spi_mode;
    uint8_t spi_bits_per_word;
    uint32_t spi_speed_hz;
    int init_calls;
    int read_calls;
    bool fail_init;
    bool fail_read;
    bool calibration_profile;
};

struct callback_state {
    int calls;
};

static int g_fake_free_calls;

static void report_expectation_failure(const char *expr, const char *file,
        int line, const char *message)
{
    fprintf(stderr, "[FAIL] %s:%d: %s", file, line, expr);
    if (message && *message)
        fprintf(stderr, " (%s)", message);
    fputc('\n', stderr);
}

#define EXPECT_TRUE(expr, message) \
    do { \
        if (!(expr)) { \
            report_expectation_failure(#expr, __FILE__, __LINE__, message); \
            return 1; \
        } \
    } while (0)

static int float_close(float a, float b)
{
    return fabsf(a - b) < 1e-4f;
}

static int fake_init(struct imu_dev *dev)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data)
        return -EINVAL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->init_calls++;
    if (priv->fail_init)
        return -EIO;

    return 0;
}

static int fake_read(struct imu_dev *dev, struct imu_data *data)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data)
        return -EINVAL;
    if (!data)
        return -EINVAL;

    priv = (struct fake_priv *)dev->priv_data;
    if (priv->fail_read)
        return -EAGAIN;

    memset(data, 0, sizeof(*data));
    priv->read_calls++;
    data->timestamp_us = 1000u + (uint64_t)priv->read_calls;

    if (priv->calibration_profile) {
        data->gyro[0] = 0.5f;
        data->gyro[1] = -1.0f;
        data->gyro[2] = 2.0f;
        data->temp = 28.5f;
        return 0;
    }

    if (strstr(priv->dev_path, "spi") != NULL) {
        data->acc[0] = 9.0f;
        data->acc[1] = 8.0f;
        data->acc[2] = 7.0f;
        data->gyro[0] = 0.1f;
        data->gyro[1] = 0.2f;
        data->gyro[2] = 0.3f;
        data->temp = 31.5f;
        return 0;
    }

    if (strstr(priv->dev_path, "uart") != NULL) {
        data->acc[0] = 0.0f;
        data->acc[1] = 0.0f;
        data->acc[2] = 9.8f;
        data->gyro[0] = 0.01f;
        data->gyro[1] = 0.02f;
        data->gyro[2] = 0.03f;
        data->quat[0] = 1.0f;
        data->temp = 26.0f;
        return 0;
    }

    data->acc[0] = 1.0f;
    data->acc[1] = 2.0f;
    data->acc[2] = 3.0f;
    data->gyro[0] = 4.0f;
    data->gyro[1] = 5.0f;
    data->gyro[2] = 6.0f;
    data->temp = 25.0f;
    return 0;
}

static void fake_free(struct imu_dev *dev)
{
    if (!dev)
        return;

    g_fake_free_calls++;
    free(dev->priv_data);
    free(dev->name);
    free(dev);
}

static const struct imu_ops fake_ops = {
    .init = fake_init,
    .read = fake_read,
    .free = fake_free,
};

static void fake_callback(struct imu_dev *dev,
        const struct imu_data *data, void *ctx)
{
    struct callback_state *state = (struct callback_state *)ctx;

    if (!dev || !data || !state)
        return;
    state->calls++;
}

static struct imu_dev *fake_create_common(const char *instance,
        const char *dev_path)
{
    struct imu_dev *dev;
    struct fake_priv *priv;

    if (!instance || !dev_path)
        return NULL;

    dev = imu_dev_alloc(instance, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = (struct fake_priv *)dev->priv_data;
    dev->ops = &fake_ops;

    strncpy(priv->dev_path, dev_path, sizeof(priv->dev_path) - 1);
    priv->dev_path[sizeof(priv->dev_path) - 1] = '\0';
    priv->fail_init = strcmp(dev_path, "/dev/fail-init") == 0;
    priv->fail_read = strcmp(dev_path, "/dev/fail-read") == 0;
    priv->calibration_profile = strcmp(dev_path, "/dev/fake-calib") == 0;

    return dev;
}

static struct imu_dev *fake_i2c_create(void *args)
{
    struct imu_args_i2c *i2c_args = (struct imu_args_i2c *)args;
    struct imu_dev *dev;
    struct fake_priv *priv;

    if (!i2c_args || !i2c_args->instance || !i2c_args->dev_path)
        return NULL;

    dev = fake_create_common(i2c_args->instance, i2c_args->dev_path);
    if (!dev)
        return NULL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->bus_arg = i2c_args->addr;
    return dev;
}

static struct imu_dev *fake_spi_create(void *args)
{
    struct imu_args_spi *spi_args = (struct imu_args_spi *)args;
    struct imu_spi_config *spi_cfg;
    struct imu_dev *dev;
    struct fake_priv *priv;

    if (!spi_args || !spi_args->instance || !spi_args->dev_path)
        return NULL;

    dev = fake_create_common(spi_args->instance, spi_args->dev_path);
    if (!dev)
        return NULL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->bus_arg = spi_args->cs_pin;

    spi_cfg = (struct imu_spi_config *)spi_args->ex_args;
    if (spi_cfg) {
        priv->spi_mode = spi_cfg->mode;
        priv->spi_bits_per_word = spi_cfg->bits_per_word;
        priv->spi_speed_hz = spi_cfg->speed_hz;
    }

    return dev;
}

static struct imu_dev *fake_uart_create(void *args)
{
    struct imu_args_uart *uart_args = (struct imu_args_uart *)args;
    struct imu_dev *dev;
    struct fake_priv *priv;

    if (!uart_args || !uart_args->instance || !uart_args->dev_path)
        return NULL;

    dev = fake_create_common(uart_args->instance, uart_args->dev_path);
    if (!dev)
        return NULL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->bus_arg = uart_args->baud;
    return dev;
}

REGISTER_IMU_DRIVER("FAKEI2C", IMU_DRV_I2C, fake_i2c_create);
REGISTER_IMU_DRIVER("FAKESPI", IMU_DRV_SPI, fake_spi_create);
REGISTER_IMU_DRIVER("FAKEUART", IMU_DRV_UART, fake_uart_create);

static int test_i2c_transform_and_offsets(void)
{
    struct imu_dev *dev;
    struct fake_priv *priv;
    struct imu_data data;
    struct callback_state callback_state = {0};
    int free_calls_before = g_fake_free_calls;
    struct imu_config cfg = {
        .mounting_matrix = {
            0.0f, 1.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f,
        },
        .acc_offset = {1.0f, 1.0f, 1.0f},
        .gyro_offset = {1.0f, 2.0f, 3.0f},
        .sample_rate = 200,
        .dlpf_freq = 50,
    };

    dev = imu_alloc_i2c("FAKEI2C:body-imu", "/dev/fake-i2c", 0x1CU, NULL);
    EXPECT_TRUE(dev != NULL, "fake i2c allocation should succeed");
    EXPECT_TRUE(strcmp(dev->name, "body-imu") == 0,
            "instance name should be split from driver name");

    imu_set_callback(dev, fake_callback, &callback_state);
    EXPECT_TRUE(dev->cb == fake_callback, "callback should be stored on device");
    EXPECT_TRUE(dev->cb_ctx == &callback_state,
            "callback context should be stored on device");

    EXPECT_TRUE(imu_init(dev, &cfg) == 0,
            "i2c init with custom config should succeed");
    EXPECT_TRUE(dev->config.sample_rate == 200U,
            "custom sample rate should be preserved");
    EXPECT_TRUE(dev->config.dlpf_freq == 50U,
            "custom dlpf should be preserved");

    priv = (struct fake_priv *)dev->priv_data;
    EXPECT_TRUE(priv != NULL, "private data should exist");
    EXPECT_TRUE(strcmp(priv->dev_path, "/dev/fake-i2c") == 0,
            "device path should be captured by fake driver");
    EXPECT_TRUE(priv->bus_arg == 0x1CU,
            "i2c address should be captured by fake driver");
    EXPECT_TRUE(priv->init_calls == 1, "driver init should run once");

    EXPECT_TRUE(imu_read(dev, &data) == 0, "i2c read should succeed");
    EXPECT_TRUE(data.timestamp_us == 1001U,
            "timestamp should come from fake driver");
    EXPECT_TRUE(float_close(data.acc[0], 1.0f), "transformed acc x mismatch");
    EXPECT_TRUE(float_close(data.acc[1], 0.0f), "transformed acc y mismatch");
    EXPECT_TRUE(float_close(data.acc[2], 2.0f), "transformed acc z mismatch");
    EXPECT_TRUE(float_close(data.gyro[0], 3.0f), "transformed gyro x mismatch");
    EXPECT_TRUE(float_close(data.gyro[1], 3.0f), "transformed gyro y mismatch");
    EXPECT_TRUE(float_close(data.gyro[2], 3.0f), "transformed gyro z mismatch");
    EXPECT_TRUE(priv->read_calls == 1, "driver read should run once");
    EXPECT_TRUE(callback_state.calls == 0,
            "core should not invoke callback during synchronous read");

    imu_free(dev);
    EXPECT_TRUE(g_fake_free_calls == free_calls_before + 1,
            "fake free should be routed through driver ops");
    printf("[PASS] i2c-transform-and-offsets\n");
    return 0;
}

static int test_spi_default_config_and_ex_args(void)
{
    struct imu_dev *dev;
    struct fake_priv *priv;
    struct imu_data data;
    int free_calls_before = g_fake_free_calls;
    struct imu_spi_config spi_cfg = {
        .mode = 3U,
        .bits_per_word = 8U,
        .speed_hz = 4000000U,
    };

    dev = imu_alloc_spi("FAKESPI:imu-spi0", "/dev/fake-spi", 7U, &spi_cfg);
    EXPECT_TRUE(dev != NULL, "fake spi allocation should succeed");
    EXPECT_TRUE(strcmp(dev->name, "imu-spi0") == 0,
            "spi instance name should be preserved");
    EXPECT_TRUE(imu_init(dev, NULL) == 0,
            "spi init with default config should succeed");
    EXPECT_TRUE(dev->config.sample_rate == 100U,
            "default sample rate should be 100Hz");
    EXPECT_TRUE(float_close(dev->config.mounting_matrix[0], 1.0f),
            "default mounting matrix xx should be identity");
    EXPECT_TRUE(float_close(dev->config.mounting_matrix[4], 1.0f),
            "default mounting matrix yy should be identity");
    EXPECT_TRUE(float_close(dev->config.mounting_matrix[8], 1.0f),
            "default mounting matrix zz should be identity");

    priv = (struct fake_priv *)dev->priv_data;
    EXPECT_TRUE(priv->bus_arg == 7U, "spi cs pin should be captured");
    EXPECT_TRUE(priv->spi_mode == 3U, "spi mode should be forwarded");
    EXPECT_TRUE(priv->spi_bits_per_word == 8U,
            "spi bits per word should be forwarded");
    EXPECT_TRUE(priv->spi_speed_hz == 4000000U,
            "spi speed should be forwarded");

    EXPECT_TRUE(imu_read(dev, &data) == 0, "spi read should succeed");
    EXPECT_TRUE(float_close(data.acc[0], 9.0f), "spi acc x mismatch");
    EXPECT_TRUE(float_close(data.gyro[2], 0.3f), "spi gyro z mismatch");
    EXPECT_TRUE(float_close(data.temp, 31.5f), "spi temp mismatch");

    imu_free(dev);
    EXPECT_TRUE(g_fake_free_calls == free_calls_before + 1,
            "spi device should use fake free op");
    printf("[PASS] spi-default-config-and-ex-args\n");
    return 0;
}

static int test_uart_gyro_calibration(void)
{
    struct imu_dev *dev;
    struct fake_priv *priv;
    struct imu_data data;
    int free_calls_before = g_fake_free_calls;

    dev = imu_alloc_uart("FAKEUART:cmp10a-imu", "/dev/fake-calib", 115200U,
            NULL);
    EXPECT_TRUE(dev != NULL, "fake uart allocation should succeed");
    EXPECT_TRUE(imu_init(dev, NULL) == 0,
            "uart init with default config should succeed");
    EXPECT_TRUE(imu_calibrate_gyro_bias(dev, 1U) == 0,
            "gyro calibration should succeed with deterministic samples");
    EXPECT_TRUE(float_close(dev->config.gyro_offset[0], 0.5f),
            "gyro offset x mismatch after calibration");
    EXPECT_TRUE(float_close(dev->config.gyro_offset[1], -1.0f),
            "gyro offset y mismatch after calibration");
    EXPECT_TRUE(float_close(dev->config.gyro_offset[2], 2.0f),
            "gyro offset z mismatch after calibration");

    EXPECT_TRUE(imu_read(dev, &data) == 0,
            "uart read after calibration should succeed");
    EXPECT_TRUE(float_close(data.gyro[0], 0.0f),
            "calibrated gyro x should be near zero");
    EXPECT_TRUE(float_close(data.gyro[1], 0.0f),
            "calibrated gyro y should be near zero");
    EXPECT_TRUE(float_close(data.gyro[2], 0.0f),
            "calibrated gyro z should be near zero");

    priv = (struct fake_priv *)dev->priv_data;
    EXPECT_TRUE(priv->bus_arg == 115200U, "uart baud should be captured");
    EXPECT_TRUE(priv->read_calls >= 2,
            "calibration plus validation should perform multiple reads");

    imu_free(dev);
    EXPECT_TRUE(g_fake_free_calls == free_calls_before + 1,
            "uart device should use fake free op");
    printf("[PASS] uart-gyro-calibration\n");
    return 0;
}

static int run_functional_tests(void)
{
    if (test_i2c_transform_and_offsets() != 0)
        return 1;
    if (test_spi_default_config_and_ex_args() != 0)
        return 1;
    if (test_uart_gyro_calibration() != 0)
        return 1;

    printf("ALL TESTS PASSED: functional\n");
    return 0;
}

static int test_invalid_alloc_and_api_inputs(void)
{
    struct imu_data data;

    EXPECT_TRUE(imu_alloc_i2c(NULL, "/dev/fake-i2c", 0x1CU, NULL) == NULL,
            "NULL i2c name should be rejected");
    EXPECT_TRUE(imu_alloc_spi("FAKESPI", NULL, 0U, NULL) == NULL,
            "NULL spi path should be rejected");
    EXPECT_TRUE(imu_alloc_uart("UNKNOWN", "/dev/fake-uart", 115200U,
                NULL) == NULL,
            "unknown uart driver should be rejected");
    EXPECT_TRUE(imu_alloc_spi("FAKEI2C:wrong-type", "/dev/fake-spi", 0U,
                NULL) == NULL,
            "driver type mismatch should be rejected");
    EXPECT_TRUE(imu_alloc_i2c("FAKEI2C:", "/dev/fake-i2c", 0x1CU,
                NULL) == NULL,
            "missing instance after colon should be rejected");
    EXPECT_TRUE(imu_alloc_uart(":imu0", "/dev/fake-uart", 115200U,
                NULL) == NULL,
            "missing driver before colon should be rejected");
    EXPECT_TRUE(imu_init(NULL, NULL) == -1,
            "NULL init should return -1");
    EXPECT_TRUE(imu_read(NULL, &data) == -1,
            "NULL read should return -1");
    EXPECT_TRUE(imu_calibrate_gyro_bias(NULL, 5U) == -1,
            "NULL calibration should return -1");

    printf("[PASS] invalid-alloc-and-api-inputs\n");
    return 0;
}

static int test_driver_failure_paths(void)
{
    struct imu_dev *dev;
    struct imu_spi_config spi_cfg = {
        .mode = 0U,
        .bits_per_word = 8U,
        .speed_hz = 1000000U,
    };
    struct imu_data data;
    int free_calls_before = g_fake_free_calls;

    dev = imu_alloc_i2c("FAKEI2C:init-failure", "/dev/fail-init", 0x1CU,
            NULL);
    EXPECT_TRUE(dev != NULL,
            "allocation before init failure should still succeed");
    EXPECT_TRUE(imu_init(dev, NULL) == -EIO,
            "driver init failure should propagate -EIO");
    imu_free(dev);

    dev = imu_alloc_spi("FAKESPI:read-failure", "/dev/fail-read", 0U,
            &spi_cfg);
    EXPECT_TRUE(dev != NULL,
            "allocation before read failure should still succeed");
    EXPECT_TRUE(imu_init(dev, NULL) == 0,
            "read failure case should still initialize");
    EXPECT_TRUE(imu_read(dev, NULL) == -EINVAL,
            "NULL output buffer should be rejected by driver read");
    EXPECT_TRUE(imu_read(dev, &data) == -EAGAIN,
            "driver read failure should propagate -EAGAIN");
    imu_free(dev);

    EXPECT_TRUE(g_fake_free_calls == free_calls_before + 2,
            "failure-path devices should still be freed through ops");
    printf("[PASS] driver-failure-paths\n");
    return 0;
}

static int test_missing_ops_device(void)
{
    struct imu_dev *dev = imu_dev_alloc("bare-dev", 0U);
    struct imu_data data;

    EXPECT_TRUE(dev != NULL, "bare device allocation should succeed");
    EXPECT_TRUE(imu_init(dev, NULL) == -1,
            "device without init op should fail init");
    EXPECT_TRUE(imu_read(dev, &data) == -1,
            "device without read op should fail read");

    imu_free(dev);
    printf("[PASS] missing-ops-device\n");
    return 0;
}

static int run_error_tests(void)
{
    if (test_invalid_alloc_and_api_inputs() != 0)
        return 1;
    if (test_driver_failure_paths() != 0)
        return 1;
    if (test_missing_ops_device() != 0)
        return 1;

    printf("ALL TESTS PASSED: error-paths\n");
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [functional|errors|all]\n", prog);
}

int main(int argc, char **argv)
{
    if (argc != 2) {
        usage(argv[0]);
        return 1;
    }

    if (strcmp(argv[1], "functional") == 0)
        return run_functional_tests();
    if (strcmp(argv[1], "errors") == 0)
        return run_error_tests();
    if (strcmp(argv[1], "all") == 0) {
        if (run_functional_tests() != 0)
            return 1;
        return run_error_tests();
    }

    usage(argv[0]);
    return 1;
}
