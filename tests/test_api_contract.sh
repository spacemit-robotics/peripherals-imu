#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
module_root="$(cd "$script_dir/.." && pwd)"
artifact_dir="${SROBOTIS_TEST_ARTIFACT_DIR:-${SROBOTIS_OUTPUT_ROOT:-$PWD/output}/test-artifacts/components/peripherals/imu/${SROBOTIS_TEST_NAME:-imu-api-contract}}"
log_dir="$artifact_dir/logs"
log_file="$log_dir/imu_api_contract.log"
build_dir="$artifact_dir/build"

mkdir -p "$log_dir" "$build_dir"

{
    echo "[info] module_root=$module_root"
    echo "[info] build_dir=$build_dir"

    cc -D_DEFAULT_SOURCE -std=c99 -Wall -Wextra -pedantic \
        -I"$module_root/include" \
        -I"$module_root/src" \
        "$module_root/src/imu_core.c" \
        "$module_root/tests/test_imu_api_contract.c" \
        -lm \
        -o "$build_dir/test_imu_api_contract"

    "$build_dir/test_imu_api_contract" functional
} | tee "$log_file"

grep -q "ALL TESTS PASSED: functional" "$log_file"
