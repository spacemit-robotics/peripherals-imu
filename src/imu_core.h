/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef IMU_CORE_H
#define IMU_CORE_H

/*
 * Private header for IMU component.
 * Keep this minimal and motor-like: args + ops + dev + driver registry.
 */

#include "../include/imu.h"
#include <stddef.h>

/* 1. 参数适配包：用于将 alloc 参数打包成 void*（同时携带 instance 名称） */
struct imu_args_i2c {
    const char *instance;
    const char *dev_path;
    uint8_t addr;
    void *ex_args;
};

struct imu_args_spi {
    const char *instance;
    const char *dev_path;
    uint32_t cs_pin;
    void *ex_args;
};

struct imu_args_uart {
    const char *instance;
    const char *dev_path;
    uint32_t baud;
    void *ex_args;
};

/* 2. 驱动类型枚举：防止类型混用 */
enum imu_driver_type {
    IMU_DRV_I2C = 0,
    IMU_DRV_SPI,
    IMU_DRV_UART,
};

/* 3. 虚函数表（驱动实现） */
struct imu_ops {
    int (*init)(struct imu_dev *dev);
    int (*read)(struct imu_dev *dev, struct imu_data *data);
    void (*free)(struct imu_dev *dev);
};

/* 4. 设备对象（私有实现，imu.h 中为不透明类型） */
struct imu_dev {
    const char *name; /* instance name */
    struct imu_config config;
    const struct imu_ops *ops;
    void *priv_data;
    imu_callback_t cb;
    void *cb_ctx;
};

/* 5. 通用工厂函数类型 */
typedef struct imu_dev *(*imu_factory_t)(void *args);

/* 6. 注册节点结构 */
struct driver_info {
    const char *name;               /* driver name */
    enum imu_driver_type type;      /* bus type */
    imu_factory_t factory;          /* create(dev args) */
    struct driver_info *next;
};

/* Core 提供的注册 API */
void imu_driver_register(struct driver_info *info);

/* 自动注册宏 (GCC/Clang Constructor) */
#define REGISTER_IMU_DRIVER(_name, _type, _factory) \
    static struct driver_info __drv_info_##_factory = { \
        .name = _name, \
        .type = _type, \
        .factory = _factory, \
        .next = 0 \
    }; \
    __attribute__((constructor)) \
    static void __auto_reg_##_factory(void) { \
        imu_driver_register(&__drv_info_##_factory); \
    }

/* Internal helper APIs */
struct imu_dev *imu_dev_alloc(const char *instance, size_t priv_size);
void imu_apply_rotation_and_offset(struct imu_dev *dev, struct imu_data *data);

#endif  /* IMU_CORE_H */

