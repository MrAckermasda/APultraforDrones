#!/usr/bin/env bash
set -e
software_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${software_script_dir}/../scripts/setup_pc.bash"
ros2 launch peripherals usb_cam.launch.py &
camera_launch_pid=$!
trap 'kill "$camera_launch_pid" 2>/dev/null || true' EXIT
ros2 run camera_calibration cameracalibrator --size=9x6 --square=0.024 \
  --no-service-check --ros-args --remap image:=/image_raw --remap camera:=/camera
