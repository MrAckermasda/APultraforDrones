#!/usr/bin/env bash
set -e
software_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${software_script_dir}/../../scripts/setup_pc.bash"
cd -- "${software_script_dir}"
# Start the original UI and backend, retaining the vendor startup delay.
(
    sleep 5
    exec xterm -T "ArmPi calibration" -e python3 "${software_script_dir}/main.py"
) &
calibration_ui_pid=$!
trap 'kill "$calibration_ui_pid" 2>/dev/null || true' EXIT
python3 "${software_script_dir}/calibration.py" "$@"
