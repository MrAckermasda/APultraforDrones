#!/usr/bin/env bash
set -e
software_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${software_script_dir}/../../../scripts/setup_pc.bash"
cd -- "${software_script_dir}"
exec python3 "${software_script_dir}/main.py" "$@"
