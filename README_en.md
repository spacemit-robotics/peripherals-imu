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
- `drv_i2c_mxc4005` - MXC4005 I2C interface accelerometer
- `drv_spi_icm42670p` - ICM-42670-P SPI interface 6-axis IMU

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
cmake -DBUILD_TESTS=ON \
  -DSROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS="drv_uart_cmp10a;drv_spi_icm42670p" ..
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

ICM-42670-P SPI examples:

```bash
./test_imu_spi -d /dev/spidev0.0 -s 1000000 -m 0 -r 100 -n 20
./test_imu_spi -d /dev/spidev0.0 -s 8000000 -r 200 -l 53 -c 3000
```

Code example (integrate into your project):

```c
#include "imu.h"

struct imu_spi_config spi_cfg = {
    .mode = 0,
    .bits_per_word = 8,
    .speed_hz = 1000000,
};

// Create SPI interface ICM-42670-P device
struct imu_dev *dev = imu_alloc_spi("icm42670p", "/dev/spidev0.0", 0, &spi_cfg);

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

The current `drv_spi_icm42670p` driver uses 4-wire SPI, Mode 0, and 8-bit word length by default according to the datasheet. It configures the accelerometer as `+-16g`, the gyroscope as `+-2000dps`, and sets ODR and DLPF based on `imu_config.sample_rate` and `imu_config.dlpf_freq`.

## Detailed Usage

> For detailed API documentation and advanced usage, please refer to the official documentation (TBD).

## FAQ

**Q: Data read returns error?**
- Check device node permissions (e.g., `/dev/ttyS1`)
- Verify baud rate configuration is correct
- For SPI devices, confirm `/dev/spidevX.Y` exists and the mode/frequency match the hardware wiring
- Ensure driver is enabled via `SROBOTIS_PERIPHERALS_IMU_ENABLED_DRIVERS`

**Q: Data orientation is incorrect?**
- Configure the sensor-to-body frame rotation matrix via `mounting_matrix`

**Q: Severe gyroscope drift?**
- Call `imu_calibrate_gyro_bias()` for bias calibration, keep the sensor stationary during calibration

## Version & Release

| Version | Date | Description |
|---------|------|-------------|
| 1.0.0 | 2026-02-28 | Initial release, supports CMP10A UART driver. |

## Contributing

Contributions are welcome. Submit an Issue to report problems, or open a Pull Request with your changes.

- **Coding style**: The C code in this component follows the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for the C-related parts. Please follow this style when writing or modifying code.
- **Pre-submit checks**: Run the repository lint script before submitting and make sure it passes:
  ```bash
  # Run from the repository root (checks the entire repository)
  bash scripts/lint/lint_cpp.sh

  # Check this component only
  bash scripts/lint/lint_cpp.sh components/peripherals/imu
  ```
  Script path: `scripts/lint/lint_cpp.sh`. If `cpplint` is not installed, install it first with `pip install cpplint` or `pipx install cpplint`.
- **Submission details**: Before submitting an Issue or PR, please describe the IMU model, connection method, and reproduction steps.

## License

Source files in this component are declared as Apache-2.0 in their headers. The `LICENSE` file in this directory shall prevail.
