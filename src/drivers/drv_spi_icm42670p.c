/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
/*
 * ICM-42670-P SPI IMU driver.
 *
 * Datasheet reference:
 * - WHO_AM_I register default: 0x67
 * - SPI transactions: first bit is R/W, next 7 bits are register address
 * - Sensor temperature: temp_c = raw / 128 + 25
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <unistd.h>

#include "../imu_core.h"

#define ICM42670P_DEFAULT_SPI_MODE       SPI_MODE_2
#define ICM42670P_DEFAULT_SPI_SPEED_HZ   1000000U
#define ICM42670P_DEFAULT_BITS_PER_WORD  8U

#define ICM42670P_WHO_AM_I_VALUE         0x67U

#define ICM42670P_REG_MCLK_RDY           0x00U
#define ICM42670P_REG_DEVICE_CONFIG      0x01U
#define ICM42670P_REG_SIGNAL_PATH_RESET  0x02U
#define ICM42670P_REG_TEMP_DATA1         0x09U
#define ICM42670P_REG_GYRO_CONFIG0       0x20U
#define ICM42670P_REG_ACCEL_CONFIG0      0x21U
#define ICM42670P_REG_GYRO_CONFIG1       0x23U
#define ICM42670P_REG_ACCEL_CONFIG1      0x24U
#define ICM42670P_REG_PWR_MGMT0          0x1FU
#define ICM42670P_REG_INTF_CONFIG0       0x35U
#define ICM42670P_REG_WHO_AM_I           0x75U

#define ICM42670P_MCLK_RDY_MASK          0x08U
#define ICM42670P_DEVICE_CONFIG_4WIRE    0x04U
#define ICM42670P_DEVICE_CONFIG_SPI12    0x01U
#define ICM42670P_SOFT_RESET_MASK        0x10U
#define ICM42670P_SENSOR_DATA_BIG_ENDIAN 0x10U
#define ICM42670P_PWR_6AXIS_LN_MODE      0x0FU

#define ICM42670P_DATA_BURST_LEN         14U
#define ICM42670P_GRAVITY_MPS2           9.80665f
#define ICM42670P_DEG_TO_RAD             0.01745329251994329577f

struct odr_entry {
    uint32_t rate_hz;
    uint8_t reg_value;
};

struct filter_entry {
    uint32_t cutoff_hz;
    uint8_t reg_value;
};

struct icm42670p_priv {
    char dev_path[64];
    uint32_t cs_pin;
    uint32_t speed_hz;
    uint8_t spi_mode;
    uint8_t bits_per_word;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
    int fd;
};

static uint64_t get_timestamp_us(void)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

static int icm42670p_spi_setup(struct icm42670p_priv *priv)
{
    uint8_t mode = priv->spi_mode;
    uint8_t bits = priv->bits_per_word;
    uint32_t speed = priv->speed_hz;

    if (ioctl(priv->fd, SPI_IOC_WR_MODE, &mode) < 0)
        return -errno;
    if (ioctl(priv->fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
        return -errno;
    if (ioctl(priv->fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
        return -errno;

    return 0;
}

static void icm42670p_dump_spi_host_config(struct icm42670p_priv *priv)
{
    uint8_t mode = 0;
    uint8_t bits = 0;
    uint32_t speed = 0;

    if (ioctl(priv->fd, SPI_IOC_RD_MODE, &mode) < 0)
        return;
    if (ioctl(priv->fd, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0)
        return;
    if (ioctl(priv->fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) < 0)
        return;

    printf("[ICM42670P] host spi config: mode=%u bits=%u speed=%uHz\n",
            (unsigned int)mode, (unsigned int)bits, (unsigned int)speed);
}

static int icm42670p_spi_write(struct icm42670p_priv *priv, uint8_t reg, uint8_t value)
{
    struct spi_ioc_transfer xfer;
    uint8_t tx[2];
    int ret;

    memset(&xfer, 0, sizeof(xfer));
    tx[0] = (uint8_t)(reg & 0x7FU);
    tx[1] = value;

    xfer.tx_buf = (uint64_t)(uintptr_t)tx;
    xfer.len = sizeof(tx);
    xfer.speed_hz = priv->speed_hz;
    xfer.bits_per_word = priv->bits_per_word;

    ret = ioctl(priv->fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0)
        return -errno;

    return 0;
}

static int icm42670p_spi_read_trace(struct icm42670p_priv *priv, uint8_t reg, uint8_t *buf,
        size_t len, const char *tag)
{
    struct spi_ioc_transfer xfer;
    uint8_t tx[1 + ICM42670P_DATA_BURST_LEN];
    uint8_t rx[1 + ICM42670P_DATA_BURST_LEN];
    int ret;
    size_t i;

    if (!buf || len == 0 || len > ICM42670P_DATA_BURST_LEN)
        return -EINVAL;

    memset(&xfer, 0, sizeof(xfer));
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    tx[0] = (uint8_t)(reg | 0x80U);

    xfer.tx_buf = (uint64_t)(uintptr_t)tx;
    xfer.rx_buf = (uint64_t)(uintptr_t)rx;
    xfer.len = len + 1U;
    xfer.speed_hz = priv->speed_hz;
    xfer.bits_per_word = priv->bits_per_word;

    ret = ioctl(priv->fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0) {
        printf("[ICM42670P] %s read reg 0x%02X failed: %s\n",
                tag ? tag : "trace", reg, strerror(errno));
        return -errno;
    }

    memcpy(buf, &rx[1], len);

    printf("[ICM42670P] %s reg=0x%02X tx:", tag ? tag : "trace", reg);
    for (i = 0; i < len + 1U; i++)
        printf(" %02X", tx[i]);
    printf(" rx:");
    for (i = 0; i < len + 1U; i++)
        printf(" %02X", rx[i]);
    printf("\n");

    return 0;
}

static int icm42670p_spi_read(struct icm42670p_priv *priv, uint8_t reg, uint8_t *buf,
        size_t len)
{
    struct spi_ioc_transfer xfer;
    uint8_t tx[1 + ICM42670P_DATA_BURST_LEN];
    uint8_t rx[1 + ICM42670P_DATA_BURST_LEN];
    int ret;

    if (!buf || len == 0 || len > ICM42670P_DATA_BURST_LEN)
        return -EINVAL;

    memset(&xfer, 0, sizeof(xfer));
    memset(tx, 0, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    tx[0] = (uint8_t)(reg | 0x80U);

    xfer.tx_buf = (uint64_t)(uintptr_t)tx;
    xfer.rx_buf = (uint64_t)(uintptr_t)rx;
    xfer.len = len + 1U;
    xfer.speed_hz = priv->speed_hz;
    xfer.bits_per_word = priv->bits_per_word;

    ret = ioctl(priv->fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret < 0)
        return -errno;

    memcpy(buf, &rx[1], len);
    return 0;
}

static uint8_t icm42670p_build_device_config(struct icm42670p_priv *priv)
{
    uint8_t value = ICM42670P_DEVICE_CONFIG_4WIRE;

    if (priv->spi_mode == SPI_MODE_1 || priv->spi_mode == SPI_MODE_2)
        value |= ICM42670P_DEVICE_CONFIG_SPI12;

    return value;
}

static int icm42670p_force_spi4(struct icm42670p_priv *priv)
{
    int ret;

    /*
     * Vendor init forces SPI 4-wire before the first read because MISO may
     * not be reliable until DEVICE_CONFIG is written once.
     */
    ret = icm42670p_spi_write(priv, ICM42670P_REG_DEVICE_CONFIG,
            ICM42670P_DEVICE_CONFIG_4WIRE);
    if (ret < 0)
        return ret;

    usleep(1000);
    return 0;
}

static int icm42670p_try_read_who_am_i(struct icm42670p_priv *priv, uint8_t *who_am_i,
        const char *tag)
{
    int ret;

    ret = icm42670p_force_spi4(priv);
    if (ret < 0)
        return ret;

    return icm42670p_spi_read_trace(priv, ICM42670P_REG_WHO_AM_I, who_am_i, 1, tag);
}

static int icm42670p_autodetect_mode(struct icm42670p_priv *priv, uint8_t *who_am_i)
{
    uint8_t original_mode = priv->spi_mode;
    const uint8_t mode_candidates[] = {
        SPI_MODE_2,
        SPI_MODE_0,
        SPI_MODE_3,
        SPI_MODE_1,
    };
    uint8_t mode;
    int ret;
    size_t i;

    printf("[ICM42670P] autodetecting SPI mode via WHO_AM_I\n");
    for (i = 0; i < sizeof(mode_candidates) / sizeof(mode_candidates[0]); i++) {
        mode = mode_candidates[i];
        priv->spi_mode = mode;
        ret = icm42670p_spi_setup(priv);
        if (ret < 0) {
            printf("[ICM42670P] detect mode=%u setup failed: %s\n",
                    (unsigned int)mode, strerror(-ret));
            continue;
        }

        ret = icm42670p_try_read_who_am_i(priv, who_am_i, "detect");
        if (ret < 0) {
            printf("[ICM42670P] detect mode=%u read failed: %s\n",
                    (unsigned int)mode, strerror(-ret));
            continue;
        }

        printf("[ICM42670P] detect mode=%u WHO_AM_I=0x%02X%s\n",
                (unsigned int)mode, *who_am_i,
                (*who_am_i == ICM42670P_WHO_AM_I_VALUE) ? " <== expected" : "");

        if (*who_am_i == ICM42670P_WHO_AM_I_VALUE)
            return 0;
    }

    priv->spi_mode = original_mode;
    ret = icm42670p_spi_setup(priv);
    if (ret < 0)
        printf("[ICM42670P] restore spi mode=%u failed: %s\n",
                (unsigned int)original_mode, strerror(-ret));

    return -ENODEV;
}

static uint8_t icm42670p_pick_odr(uint32_t sample_rate)
{
    static const struct odr_entry k_odr_table[] = {
        {1600U, 0x05U},
        {800U, 0x06U},
        {400U, 0x07U},
        {200U, 0x08U},
        {100U, 0x09U},
        {50U, 0x0AU},
        {25U, 0x0BU},
        {12U, 0x0CU},
    };
    uint32_t best_delta;
    uint8_t best_reg;
    size_t i;

    if (sample_rate == 0U)
        sample_rate = 100U;

    best_reg = k_odr_table[0].reg_value;
    best_delta = (sample_rate > k_odr_table[0].rate_hz)
            ? (sample_rate - k_odr_table[0].rate_hz)
            : (k_odr_table[0].rate_hz - sample_rate);

    for (i = 1; i < sizeof(k_odr_table) / sizeof(k_odr_table[0]); i++) {
        uint32_t delta = (sample_rate > k_odr_table[i].rate_hz)
                ? (sample_rate - k_odr_table[i].rate_hz)
                : (k_odr_table[i].rate_hz - sample_rate);

        if (delta < best_delta) {
            best_delta = delta;
            best_reg = k_odr_table[i].reg_value;
        }
    }

    return best_reg;
}

static uint8_t icm42670p_pick_filter(uint32_t cutoff_hz)
{
    static const struct filter_entry k_filter_table[] = {
        {16U, 0x07U},
        {25U, 0x06U},
        {34U, 0x05U},
        {53U, 0x04U},
        {73U, 0x03U},
        {121U, 0x02U},
        {180U, 0x01U},
    };
    uint32_t best_delta;
    uint8_t best_reg;
    size_t i;

    if (cutoff_hz == 0U)
        return 0xFFU;

    best_reg = k_filter_table[0].reg_value;
    best_delta = (cutoff_hz > k_filter_table[0].cutoff_hz)
            ? (cutoff_hz - k_filter_table[0].cutoff_hz)
            : (k_filter_table[0].cutoff_hz - cutoff_hz);

    for (i = 1; i < sizeof(k_filter_table) / sizeof(k_filter_table[0]); i++) {
        uint32_t delta = (cutoff_hz > k_filter_table[i].cutoff_hz)
                ? (cutoff_hz - k_filter_table[i].cutoff_hz)
                : (k_filter_table[i].cutoff_hz - cutoff_hz);

        if (delta < best_delta) {
            best_delta = delta;
            best_reg = k_filter_table[i].reg_value;
        }
    }

    return best_reg;
}

static int16_t icm42670p_be16(const uint8_t *buf)
{
    return (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
}

static int icm42670p_init(struct imu_dev *dev)
{
    struct icm42670p_priv *priv = dev->priv_data;
    uint8_t mclk_rdy = 0;
    uint8_t who_am_i = 0;
    uint8_t odr_reg = icm42670p_pick_odr(dev->config.sample_rate);
    uint8_t filter_reg = icm42670p_pick_filter(dev->config.dlpf_freq);
    int ret;

    printf("[ICM42670P] init: %s, dev=%s, cs=%u, speed=%uHz, mode=%u\n",
            dev->name, priv->dev_path, (unsigned int)priv->cs_pin,
            (unsigned int)priv->speed_hz, (unsigned int)priv->spi_mode);

    priv->fd = open(priv->dev_path, O_RDWR);
    if (priv->fd < 0) {
        printf("[ICM42670P] open %s failed: %s\n",
                priv->dev_path, strerror(errno));
        return -errno;
    }

    ret = icm42670p_spi_setup(priv);
    if (ret < 0) {
        printf("[ICM42670P] spi setup failed: %s\n", strerror(-ret));
        close(priv->fd);
        priv->fd = -1;
        return ret;
    }
    icm42670p_dump_spi_host_config(priv);

    ret = icm42670p_spi_read_trace(priv, ICM42670P_REG_MCLK_RDY, &mclk_rdy, 1,
            "precheck");
    if (ret == 0) {
        printf("[ICM42670P] MCLK_RDY raw=0x%02X ready=%u\n",
                mclk_rdy, (unsigned int)((mclk_rdy & ICM42670P_MCLK_RDY_MASK) != 0U));
    }

    ret = icm42670p_try_read_who_am_i(priv, &who_am_i, "precheck");
    if (ret < 0) {
        printf("[ICM42670P] failed to read WHO_AM_I\n");
        close(priv->fd);
        priv->fd = -1;
        return ret;
    }
    if (who_am_i != ICM42670P_WHO_AM_I_VALUE) {
        printf("[ICM42670P] unexpected WHO_AM_I: 0x%02X\n", who_am_i);
        ret = icm42670p_autodetect_mode(priv, &who_am_i);
        if (ret < 0) {
            close(priv->fd);
            priv->fd = -1;
            return ret;
        }
        printf("[ICM42670P] using autodetected SPI mode %u\n",
                (unsigned int)priv->spi_mode);
    }

    ret = icm42670p_spi_write(priv, ICM42670P_REG_SIGNAL_PATH_RESET,
            ICM42670P_SOFT_RESET_MASK);
    if (ret < 0)
        goto err_out;

    usleep(1000);

    ret = icm42670p_force_spi4(priv);
    if (ret < 0) {
        printf("[ICM42670P] failed to restore SPI4 after reset\n");
        goto err_out;
    }

    ret = icm42670p_spi_write(priv, ICM42670P_REG_DEVICE_CONFIG,
            icm42670p_build_device_config(priv));
    if (ret < 0)
        goto err_out;

    ret = icm42670p_spi_write(priv, ICM42670P_REG_INTF_CONFIG0,
            ICM42670P_SENSOR_DATA_BIG_ENDIAN);
    if (ret < 0)
        goto err_out;

    ret = icm42670p_spi_write(priv, ICM42670P_REG_GYRO_CONFIG0, odr_reg);
    if (ret < 0)
        goto err_out;

    ret = icm42670p_spi_write(priv, ICM42670P_REG_ACCEL_CONFIG0, odr_reg);
    if (ret < 0)
        goto err_out;

    if (filter_reg != 0xFFU) {
        ret = icm42670p_spi_write(priv, ICM42670P_REG_GYRO_CONFIG1, filter_reg);
        if (ret < 0)
            goto err_out;

        ret = icm42670p_spi_write(priv, ICM42670P_REG_ACCEL_CONFIG1, filter_reg);
        if (ret < 0)
            goto err_out;
    }
    ret = icm42670p_spi_write(priv, ICM42670P_REG_PWR_MGMT0,
            ICM42670P_PWR_6AXIS_LN_MODE);
    if (ret < 0)
        goto err_out;

    usleep(50000);

    printf("[ICM42670P] ready, odr_code=0x%02X, dlpf_code=%s\n",
            odr_reg, (filter_reg == 0xFFU) ? "default" : "configured");
    return 0;

err_out:
    printf("[ICM42670P] init failed: %s\n", strerror(-ret));
    close(priv->fd);
    priv->fd = -1;
    return ret;
}

static int icm42670p_read(struct imu_dev *dev, struct imu_data *data)
{
    struct icm42670p_priv *priv = dev->priv_data;
    uint8_t raw[ICM42670P_DATA_BURST_LEN];
    int16_t temp_raw;
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int ret;

    if (!data || priv->fd < 0)
        return -EINVAL;

    memset(data, 0, sizeof(*data));

    ret = icm42670p_spi_read(priv, ICM42670P_REG_TEMP_DATA1, raw, sizeof(raw));
    if (ret < 0)
        return ret;

    temp_raw = icm42670p_be16(&raw[0]);
    accel_raw[0] = icm42670p_be16(&raw[2]);
    accel_raw[1] = icm42670p_be16(&raw[4]);
    accel_raw[2] = icm42670p_be16(&raw[6]);
    gyro_raw[0] = icm42670p_be16(&raw[8]);
    gyro_raw[1] = icm42670p_be16(&raw[10]);
    gyro_raw[2] = icm42670p_be16(&raw[12]);

    data->timestamp_us = get_timestamp_us();
    data->temp = ((float)temp_raw / 128.0f) + 25.0f;
    data->acc[0] = ((float)accel_raw[0] / priv->accel_lsb_per_g) * ICM42670P_GRAVITY_MPS2;
    data->acc[1] = ((float)accel_raw[1] / priv->accel_lsb_per_g) * ICM42670P_GRAVITY_MPS2;
    data->acc[2] = ((float)accel_raw[2] / priv->accel_lsb_per_g) * ICM42670P_GRAVITY_MPS2;
    data->gyro[0] = ((float)gyro_raw[0] / priv->gyro_lsb_per_dps) * ICM42670P_DEG_TO_RAD;
    data->gyro[1] = ((float)gyro_raw[1] / priv->gyro_lsb_per_dps) * ICM42670P_DEG_TO_RAD;
    data->gyro[2] = ((float)gyro_raw[2] / priv->gyro_lsb_per_dps) * ICM42670P_DEG_TO_RAD;

    return 0;
}

static void icm42670p_free(struct imu_dev *dev)
{
    struct icm42670p_priv *priv;

    if (!dev)
        return;

    priv = dev->priv_data;
    if (priv && priv->fd >= 0) {
        close(priv->fd);
        priv->fd = -1;
    }

    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free((void *)dev->name);
    free(dev);
}

static const struct imu_ops icm42670p_ops = {
    .init = icm42670p_init,
    .read = icm42670p_read,
    .free = icm42670p_free,
};

static struct imu_dev *icm42670p_create(void *args)
{
    struct imu_args_spi *a = (struct imu_args_spi *)args;
    const struct imu_spi_config *cfg;
    struct imu_dev *dev;
    struct icm42670p_priv *priv;

    if (!a || !a->instance || !a->dev_path)
        return NULL;

    dev = imu_dev_alloc(a->instance, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = dev->priv_data;
    dev->ops = &icm42670p_ops;

    strncpy(priv->dev_path, a->dev_path, sizeof(priv->dev_path) - 1);
    priv->dev_path[sizeof(priv->dev_path) - 1] = '\0';
    priv->cs_pin = a->cs_pin;
    priv->speed_hz = ICM42670P_DEFAULT_SPI_SPEED_HZ;
    priv->spi_mode = ICM42670P_DEFAULT_SPI_MODE;
    priv->bits_per_word = ICM42670P_DEFAULT_BITS_PER_WORD;
    priv->accel_lsb_per_g = 2048.0f;
    priv->gyro_lsb_per_dps = 16.4f;
    priv->fd = -1;

    cfg = (const struct imu_spi_config *)a->ex_args;
    if (cfg) {
        if (cfg->speed_hz > 0U)
            priv->speed_hz = cfg->speed_hz;
        if (cfg->bits_per_word > 0U)
            priv->bits_per_word = cfg->bits_per_word;
        priv->spi_mode = cfg->mode;
    }

    return dev;
}

REGISTER_IMU_DRIVER("icm42670p", IMU_DRV_SPI, icm42670p_create);
