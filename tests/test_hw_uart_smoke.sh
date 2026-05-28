#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/peripherals/imu/${SROBOTIS_TEST_NAME:-imu-uart-hardware-smoke}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/imu_uart_hardware_smoke.log"
build_dir="$artifact_dir/build"

: "${IMU_TEST_DEV_PATH:?Set IMU_TEST_DEV_PATH to the UART device path, e.g. /dev/ttyUSB0}"
imu_test_baud="${IMU_TEST_BAUD:-115200}"
imu_test_rate="${IMU_TEST_RATE:-10}"
imu_test_samples="${IMU_TEST_SAMPLES:-3}"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"
    echo "[info] dev=$IMU_TEST_DEV_PATH baud=$imu_test_baud rate=$imu_test_rate samples=$imu_test_samples"

    cc -D_DEFAULT_SOURCE -std=c99 -Wall -Wextra -pedantic \
        -I"$module_root/include" \
        -I"$module_root/src" \
        "$module_root/src/imu_core.c" \
        "$module_root/src/drivers/drv_uart_cmp10a.c" \
        "$module_root/tests/test_imu_uart_hardware_smoke.c" \
        -lm \
        -o "$build_dir/test_imu_uart_hardware_smoke"

    "$build_dir/test_imu_uart_hardware_smoke" \
        "$IMU_TEST_DEV_PATH" \
        "$imu_test_baud" \
        "$imu_test_rate" \
        "$imu_test_samples"
} | tee "$log_file"

grep -q "IMU initialized successfully" "$log_file"
grep -q "Total samples: $imu_test_samples" "$log_file"
awk -v expected="$imu_test_samples" '$1 ~ /^[0-9]+$/ {count++} END {exit (count >= expected) ? 0 : 1}' "$log_file"
