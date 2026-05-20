#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
build_dir="${repo_root}/.pio/host_tests"

mkdir -p "${build_dir}"

compile_and_run() {
  local test_name="$1"
  shift
  local test_bin="${build_dir}/${test_name}"

  g++ \
    -std=c++17 \
    -Wall \
    -Wextra \
    -Werror \
    -I"${repo_root}/src" \
    "$@" \
    -o "${test_bin}"

  "${test_bin}"
}

compile_and_run \
  test_motor_response_model \
  "${repo_root}/src/motor/motor_response_model.cpp" \
  "${repo_root}/test/host/test_motor_response_model.cpp"

compile_and_run \
  test_motor_command_parser \
  "${repo_root}/src/bridge/motor_command_parser.cpp" \
  "${repo_root}/test/host/test_motor_command_parser.cpp"

compile_and_run \
  test_encoder_rpm_estimator \
  "${repo_root}/src/motor/encoder_rpm_estimator.cpp" \
  "${repo_root}/test/host/test_encoder_rpm_estimator.cpp"

compile_and_run \
  test_speed_pid \
  "${repo_root}/src/motor/speed_pid.cpp" \
  "${repo_root}/test/host/test_speed_pid.cpp"

compile_and_run \
  test_single_motor_control \
  "${repo_root}/src/motor/speed_pid.cpp" \
  "${repo_root}/src/motor/single_motor_control.cpp" \
  "${repo_root}/test/host/test_single_motor_control.cpp"
