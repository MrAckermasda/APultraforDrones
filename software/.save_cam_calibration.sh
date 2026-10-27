#!/usr/bin/env bash
set -e
software_script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "${software_script_dir}/../scripts/setup_pc.bash"
python3 - "$ARMPI_WS" "${1:-/tmp/calibrationdata.tar.gz}" <<'PYCODE'
import sys
import tarfile
from pathlib import Path
import yaml

target = Path(sys.argv[1]) / 'src/peripherals/config/camera_usb_info.yaml'
with tarfile.open(sys.argv[2], 'r:gz') as archive:
    members = [m for m in archive.getmembers() if Path(m.name).name == 'ost.yaml' and m.isfile()]
    if len(members) != 1:
        raise RuntimeError('Expected exactly one ost.yaml calibration file')
    data = yaml.safe_load(archive.extractfile(members[0]))
if not isinstance(data, dict) or 'camera_matrix' not in data:
    raise RuntimeError('Invalid camera calibration YAML')
data['camera_name'] = 'usb_cam'
if target.exists():
    backup = target.with_suffix('.yaml.before-camera-calibration')
    if not backup.exists(): backup.write_bytes(target.read_bytes())
target.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
print(f'Saved: {target}')
PYCODE
