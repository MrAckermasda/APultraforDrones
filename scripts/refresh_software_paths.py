#!/usr/bin/env python3
"""Refresh local desktop/dataset paths after moving the workspace; no device access."""
from pathlib import Path
import re
import shutil
import yaml

ROOT = Path(__file__).resolve().parents[1]
BACKUP = ROOT / '.migration-backup/2026-10-27-software-paths'
DESKTOPS = {
    'software/armpi_ultra_control/arm.desktop': ('software/armpi_ultra_control/arm.sh', 'software/armpi_ultra_control/resources/arm.ico'),
    'software/calibration/armpi ultra位置微调工具.desktop': ('software/calibration/tool.sh', 'software/calibration/icon.png'),
    'software/collect_picture/collect_picture.desktop': ('software/collect_picture/collect_picture.sh', 'software/collect_picture/resources/app.png'),
    'software/lab_tool/lab_tool.desktop': ('software/lab_tool/lab_tool.sh', 'software/lab_tool/resources/lab.ico'),
}


def desktop_arg(value):
    # Exec field uses desktop-entry escaping, not shell variable expansion.
    value = str(value).replace('%', '%%')
    value = value.replace('\\', '\\\\\\\\')
    for char in ('"', '`', '$'):
        value = value.replace(char, '\\\\' + char)
    return '"' + value + '"'


def refresh(root=ROOT, backup=BACKUP):
    changed = []

    def write(relative, content):
        path = root / relative
        if path.read_text() == content:
            return
        old = backup / relative
        old.parent.mkdir(parents=True, exist_ok=True)
        if not old.exists():
            shutil.copy2(path, old)
        path.write_text(content)
        changed.append(str(relative))

    for filename, (entry, icon) in DESKTOPS.items():
        text = (root / filename).read_text()
        text = re.sub(r'^(?:Encoding|MultipleArgs)=.*\n', '', text, flags=re.M)
        text = text.replace('Categories=Application;Development;', 'Categories=Development;')
        text = re.sub(r'^Exec=.*$', lambda _: f'Exec=bash {desktop_arg(root / entry)}', text, flags=re.M)
        text = re.sub(r'^Icon=.*$', lambda _: 'Icon=' + str(root / icon), text, flags=re.M)
        write(Path(filename), text)

    for relative in ('software/data/data.yaml', 'software/collect_picture/my_data/data.yaml'):
        path = root / relative
        data = yaml.safe_load(path.read_text())
        data['path'] = str(path.parent)
        if 'collect_picture' in relative:
            data['train'] = 'ImageSets/train.txt'
            data['val'] = 'ImageSets/val.txt'
        write(Path(relative), yaml.safe_dump(data, allow_unicode=True, sort_keys=False))

    # Preserve list order and entries; use paths relative to the list file.
    for path in (root / 'software/collect_picture/my_data/ImageSets').glob('*.txt'):
        text = path.read_text()
        text = re.sub(r'(?:/[^\n]*)?/software/collect_picture/my_data/', './../', text)
        write(path.relative_to(root), text)

    if changed:
        manifest = backup / 'changed-paths.txt'
        previous = manifest.read_text().splitlines() if manifest.exists() else []
        manifest.parent.mkdir(parents=True, exist_ok=True)
        manifest.write_text('\n'.join(sorted(set(previous + changed))) + '\n')
    return changed


if __name__ == '__main__':
    changed = refresh()
    print(f'Refreshed {len(changed)} files in {ROOT}')
    print('No desktop applications were installed or started. Rerun after relocating the workspace.')
