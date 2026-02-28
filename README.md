# IMU 组件

## 项目简介

IMU（惯性测量单元）组件提供统一的传感器驱动抽象层，用于读取加速度计、陀螺仪、磁力计等惯性传感器数据。解决多种 IMU 硬件接口差异问题，为上层应用提供一致的数据访问接口。

## 功能特性

**支持：**
- 多种通信接口：I2C、SPI、UART
- 传感器数据读取：加速度、角速度、磁场、四元数、温度
- 传感器坐标系变换（mounting matrix）
- 陀螺仪零偏校准
- 回调模式数据推送
- 可配置采样率和低通滤波器

**当前支持的驱动：**
- `drv_uart_cmp10a` - CMP10A UART 接口 IMU


## 快速开始

### 环境准备

- CMake >= 3.10
- C99 编译器（GCC/Clang）
- 目标硬件需连接对应 IMU 传感器

### 构建编译

脱离 SDK 单独构建：

```bash
cd peripherals-imu
mkdir build && cd build
cmake -DBUILD_TESTS=ON -DSROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS="drv_uart_cmp10a" ..
make
```

### 运行示例

构建时启用 `BUILD_TESTS=ON` 后会生成测试程序 `test_imu_uart`，可直接运行：

```bash
# 指定设备与波特率运行（CMP10A 常用 115200）
./test_imu_uart -d /dev/ttyS1 -b 115200

# 可选参数：-r 打印频率(Hz)，-c 陀螺仪校准时长(ms)，-n 采样次数，-h 帮助
./test_imu_uart -d /dev/ttyUSB0 -b 115200 -r 10 -c 3000 -n 100
```


代码示例（集成到自己的项目）：

```c
#include "imu.h"

// 创建 UART 接口 IMU 设备
struct imu_dev *dev = imu_alloc_uart("cmp10a", "/dev/ttyS1", 115200, NULL);

// 初始化
struct imu_config cfg = {
    .mounting_matrix = {1,0,0, 0,1,0, 0,0,1},
    .sample_rate = 100,
    .dlpf_freq = 50
};
imu_init(dev, &cfg);

// 读取数据
struct imu_data data;
imu_read(dev, &data);
printf("acc: %.2f, %.2f, %.2f\n", data.acc[0], data.acc[1], data.acc[2]);

// 释放资源
imu_free(dev);
```

## 详细使用

> 详细 API 文档和高级用法请参考官方文档（待补充）。

## 常见问题

**Q: 读取数据返回错误？**
- 检查设备节点权限（如 `/dev/ttyS1`）
- 确认波特率配置正确
- 确认驱动已通过 `SROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS` 启用

**Q: 数据方向不对？**
- 通过 `mounting_matrix` 配置传感器到机体坐标系的旋转矩阵

**Q: 陀螺仪漂移严重？**
- 调用 `imu_calibrate_gyro_bias()` 进行零偏校准，校准期间保持静止

## 版本与发布

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.1.0 | 2026-02-28 | 初始版本，支持 CMP10A UART 驱动 |

## 贡献方式

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/xxx`)
3. 提交更改 (`git commit -m 'Add xxx'`)
4. 推送分支 (`git push origin feature/xxx`)
5. 创建 Pull Request

## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
