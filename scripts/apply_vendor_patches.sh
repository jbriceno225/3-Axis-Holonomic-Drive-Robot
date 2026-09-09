#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

apply_patch_once() {
  local target_dir="$1"
  local patch_file="$2"
  shift 2
  local apply_args=("$@")

  if git -C "${target_dir}" apply "${apply_args[@]}" --reverse --check "${patch_file}" >/dev/null 2>&1; then
    printf 'Already applied: %s\n' "$(basename "${patch_file}")"
  elif git -C "${target_dir}" apply "${apply_args[@]}" --check "${patch_file}"; then
    git -C "${target_dir}" apply "${apply_args[@]}" "${patch_file}"
    printf 'Applied: %s\n' "$(basename "${patch_file}")"
  else
    printf 'ERROR: %s does not apply cleanly in %s\n' \
      "$(basename "${patch_file}")" "${target_dir}" >&2
    return 1
  fi
}

git -C "${ROOT_DIR}" submodule update --init --recursive

# Robot-specific port, frame, and SLAM settings live in kiwi_bringup. These
# overlays contain only source-build compatibility fixes for the pinned LD19
# driver and SDK.
apply_patch_once \
  "${ROOT_DIR}/src/ldlidar_ros2" \
  "${ROOT_DIR}/patches/ldlidar_ros2.patch"

apply_patch_once \
  "${ROOT_DIR}/src/ldlidar_ros2/sdk" \
  "${ROOT_DIR}/patches/ldlidar_sdk.patch"
