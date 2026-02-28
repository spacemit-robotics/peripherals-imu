# IMU Component

## Introduction

The IMU (Inertial Measurement Unit) component provides a unified sensor driver abstraction layer for reading data from accelerometers, gyroscopes, magnetometers, and other inertial sensors. It addresses the differences between various IMU hardware interfaces and provides a consistent data access interface for upper-layer applications.

## Features

**Supported:**
- Multiple communication interfaces: I2C, SPI, UART
- Sensor data reading: acceleration, angular velocity, magnetic field, quaternion, temperature
- Sensor coordinate transformation (mounting matrix)
- Gyroscope bias calibration
- Callback-based data push
- Configurable sample rate and low-pass filter

**Currently supported drivers:**
- `drv_uart_cmp10a` - CMP10A UART interface IMU


## Quick Start

### Prerequisites

- CMake >= 3.10
- C99 compiler (GCC/Clang)
- Target hardware with connected IMU sensor

### Build

Standalone build (without SDK):

```bash
cd peripherals-imu
mkdir build && cd build
cmake -DBUILD_TESTS=ON -DSROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS="drv_uart_cmp10a" ..
make
```

### Usage Example

With `BUILD_TESTS=ON`, the test program `test_imu_uart` is built. Run it directly:

```bash
# Run with device and baud (CMP10A typically uses 115200)
./test_imu_uart -d /dev/ttyS1 -b 115200

# Optional: -r print rate (Hz), -c gyro calibration duration (ms), -n sample count, -h help
./test_imu_uart -d /dev/ttyUSB0 -b 115200 -r 10 -c 3000 -n 100
```

Code example (integrate into your project):

```c
#include "imu.h"

// Create UART interface IMU device
struct imu_dev *dev = imu_alloc_uart("cmp10a", "/dev/ttyS1", 115200, NULL);

// Initialize
struct imu_config cfg = {
    .mounting_matrix = {1,0,0, 0,1,0, 0,0,1},
    .sample_rate = 100,
    .dlpf_freq = 50
};
imu_init(dev, &cfg);

// Read data
struct imu_data data;
imu_read(dev, &data);
printf("acc: %.2f, %.2f, %.2f\n", data.acc[0], data.acc[1], data.acc[2]);

// Free resources
imu_free(dev);
```

## Detailed Usage

> For detailed API documentation and advanced usage, please refer to the official documentation (TBD).

## FAQ

**Q: Data read returns error?**
- Check device node permissions (e.g., `/dev/ttyS1`)
- Verify baud rate configuration is correct
- Ensure driver is enabled via `SROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS`

**Q: Data orientation is incorrect?**
- Configure the sensor-to-body frame rotation matrix via `mounting_matrix`

**Q: Severe gyroscope drift?**
- Call `imu_calibrate_gyro_bias()` for bias calibration, keep the sensor stationary during calibration

## Version & Release

| Version | Date | Description |
|---------|------|-------------|
| 1.0.0 | 2024-01 | Initial release, supports CMP10A UART driver |

## Contributing

1. Fork this repository
2. Create a feature branch (`git checkout -b feature/xxx`)
3. Commit your changes (`git commit -m 'Add xxx'`)
4. Push to the branch (`git push origin feature/xxx`)
5. Create a Pull Request

## License

Source files in this component are declared as Apache-2.0 in their headers. The `LICENSE` file in this directory shall prevail.
