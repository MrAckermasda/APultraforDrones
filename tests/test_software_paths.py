"""Check software path migration without importing GUIs or opening devices."""
import ast
from pathlib import Path
import runpy
import shutil
import subprocess

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
BACKUP = ROOT / '.migration-backup/2026-10-27-software-paths'
ACTIVE_DIRS = ('armpi_ultra_control', 'calibration', 'lab_tool', 'collect_picture',
               'Servo_upper_computer', 'servo_tool', 'hand2cam_tf_matrix_software', 'data', 'yolov11')


def test_source_relative_paths_follow_workspace(tmp_path):
    relocated = tmp_path / 'another user' / 'arm workspace'
    count = 0
    for relative in (BACKUP / 'changed-paths.txt').read_text().splitlines():
        path = ROOT / relative
        if path.suffix != '.py':
            continue
        tree = ast.parse(path.read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Subscript) and ast.unparse(node.value) == 'Path(__file__).resolve().parents':
                expression = compile(ast.Expression(node), str(path), 'eval')
                assert eval(expression, {'Path': Path, '__file__': str(relocated / relative)}) == relocated
                count += 1
    assert count >= 20


def test_known_resource_paths_resolve():
    targets = {
        'software/calibration/main.py': 'src/app/config/calibration.yaml',
        'software/calibration/calibration.py': 'src/peripherals/config/camera_info.yaml',
        'software/lab_tool/main.py': 'src/app/config/lab_config.yaml',
        'software/hand2cam_tf_matrix_software/calibration/main.py': 'src/peripherals/config/',
        'software/Servo_upper_computer/yaml_handle.py': 'software/Servo_upper_computer/servo_config.yaml',
    }
    for relative, resource in targets.items():
        source = (ROOT / relative).read_text()
        expressions = [n for n in ast.walk(ast.parse(source)) if isinstance(n, ast.Call)
                       and isinstance(n.func, ast.Name) and n.func.id == 'str'
                       and resource in ast.unparse(n)]
        assert expressions, relative
        value = eval(compile(ast.Expression(expressions[0]), relative, 'eval'),
                     {'Path': Path, '__file__': str(ROOT / relative)})
        assert Path(value) == ROOT / resource
        assert Path(value).exists()


@pytest.mark.parametrize('relative', [
    'software/armpi_ultra_control/arm.sh',
    'software/lab_tool/lab_tool.sh',
    'software/collect_picture/collect_picture.sh',
    'software/servo_tool/config/servo_tool.sh',
    'software/hand2cam_tf_matrix_software/calibration/hand2cam.sh',
])
def test_launchers_in_relocated_workspace_without_python_execution(tmp_path, relative):
    root = tmp_path / 'workspace with spaces'
    script = root / relative
    script.parent.mkdir(parents=True)
    shutil.copy2(ROOT / relative, script)
    setup = root / 'scripts/setup_pc.bash'
    setup.parent.mkdir()
    # A shell function shadows python3. Removing exec in the test copy only lets
    # us check the argument vector without executing any application code.
    setup.write_text('python3() { printf "%s\\n" "$@"; }\n')
    script.write_text(script.read_text().replace('exec python3', 'python3'))
    result = subprocess.run(['bash', str(script), '--test-argument'], cwd=tmp_path,
                            capture_output=True, text=True, check=True)
    args = result.stdout.splitlines()
    assert Path(args[0]).resolve().is_relative_to(root)
    assert args[0].endswith('main.py')
    assert args[1] == '--test-argument'


def test_bash_syntax():
    for path in (ROOT / 'software').rglob('*.sh'):
        if any(name in path.parts for name in ('ultralytics', 'roLabelImg')):
            continue
        subprocess.run(['bash', '-n', str(path)], check=True, capture_output=True)


def test_refresh_is_idempotent_and_desktops_are_valid():
    refresh = runpy.run_path(str(ROOT / 'scripts/refresh_software_paths.py'))
    assert refresh['refresh']() == []
    for relative, (entry, icon) in refresh['DESKTOPS'].items():
        assert (ROOT / entry).is_file()
        assert (ROOT / icon).is_file()
        if shutil.which('desktop-file-validate'):
            subprocess.run(['desktop-file-validate', str(ROOT / relative)], check=True, capture_output=True)


def test_dataset_order_and_labels_preserved():
    relative = Path('software/collect_picture/my_data/data.yaml')
    before = yaml.safe_load((BACKUP / relative).read_text())
    after = yaml.safe_load((ROOT / relative).read_text())
    assert before['names'] == after['names']
    assert before['nc'] == after['nc']
    for split in ('train', 'val'):
        path = Path(after['path']) / after[split]
        original = (BACKUP / path.relative_to(ROOT)).read_text().splitlines()
        current = path.read_text().splitlines()
        assert len(original) == len(current)
        assert [Path(s).name for s in original] == [Path(s).name for s in current]
        assert all((path.parent / s).resolve().is_file() for s in current)


def test_no_old_runtime_paths_or_zsh_rc():
    for directory in ACTIVE_DIRS:
        for p in (ROOT / 'software' / directory).rglob('*'):
            if not p.is_file() or p.suffix not in ('.py', '.sh', '.desktop', '.yaml') or 'runs' in p.parts:
                continue
            text = p.read_text()
            for old in ('/home/ubuntu/', '/home/hiwonder/', '/home/pi/', '/home/lyl/software/', '.zshrc'):
                assert old not in text, (str(p), old)


def test_copied_serial_sdks_use_environment_without_real_serial(monkeypatch):
    from unittest.mock import patch
    monkeypatch.setenv('ARMPI_SERIAL_PORT', '/dev/test-arm-never-opened')
    monkeypatch.setenv('ARMPI_BAUDRATE', '115200')
    for relative in ('software/Servo_upper_computer/ros_robot_controller_sdk.py',
                     'software/hand2cam_tf_matrix_software/calibration/ros_robot_controller_sdk.py'):
        with patch('serial.Serial') as serial, patch('threading.Thread'), patch('time.sleep'):
            module = runpy.run_path(str(ROOT / relative))
            module['Board']()
            serial.assert_called_once_with(None, 115200, timeout=5)
            serial.return_value.setPort.assert_called_once_with('/dev/test-arm-never-opened')


def test_camera_calibration_save_is_independent_of_cwd_and_keeps_backup(tmp_path):
    import io
    import tarfile
    root = tmp_path / 'camera workspace'
    script = root / 'software/.save_cam_calibration.sh'
    script.parent.mkdir(parents=True)
    shutil.copy2(ROOT / 'software/.save_cam_calibration.sh', script)
    setup = root / 'scripts/setup_pc.bash'
    setup.parent.mkdir()
    setup.write_text('export ARMPI_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"\n')
    target = root / 'src/peripherals/config/camera_usb_info.yaml'
    target.parent.mkdir(parents=True)
    target.write_text('camera_name: previous\n')
    archive_path = tmp_path / 'synthetic calibration.tar.gz'
    original = {'camera_name': 'narrow_stereo', 'camera_matrix': {'rows': 3, 'cols': 3, 'data': list(range(9))}}
    content = yaml.safe_dump(original).encode()
    with tarfile.open(archive_path, 'w:gz') as archive:
        info = tarfile.TarInfo('ost.yaml')
        info.size = len(content)
        archive.addfile(info, io.BytesIO(content))
    subprocess.run(['bash', str(script), str(archive_path)], cwd=tmp_path, check=True, capture_output=True)
    saved = yaml.safe_load(target.read_text())
    assert saved['camera_name'] == 'usb_cam'
    assert saved['camera_matrix'] == original['camera_matrix']
    assert target.with_suffix('.yaml.before-camera-calibration').read_text() == 'camera_name: previous\n'
