#!/usr/bin/env python3
"""Read-only portability report; never opens a device or imports vendor extensions."""
import argparse
import importlib.metadata
import os
import platform
from pathlib import Path
import struct
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--core-only', action='store_true', help='Only inspect driver binaries')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    arch = platform.machine()
    expected = {'x86_64': 62, 'aarch64': 183, 'armv7l': 40}.get(arch)
    print(f'Workspace: {root}\nCPU: {arch}\nPython: {platform.python_version()}')
    print(f'ROS_DISTRO: {os.environ.get("ROS_DISTRO", "not sourced")}')
    for package in ('pyserial', 'numpy', 'PyYAML', 'setuptools', 'transforms3d'):
        try:
            version = importlib.metadata.version(package)
        except importlib.metadata.PackageNotFoundError:
            version = 'MISSING'
        print(f'{package}: {version}')
    port = Path(os.environ.get('ARMPI_SERIAL_PORT', '/dev/rrc'))
    print(f'Serial: {port}; exists={port.exists()}; accessible={os.access(port, os.R_OK | os.W_OK)}')
    print('Device identity/protocol and firmware have NOT been verified.')
    mismatches = 0
    search = root / ('src/driver' if args.core_only else 'src')
    for library in sorted(search.rglob('*.so')):
        with library.open('rb') as stream:
            header = stream.read(20)
        if len(header) < 20 or header[:4] != b'\x7fELF':
            print(f'UNKNOWN binary: {library.relative_to(root)}')
            mismatches += 1
            continue
        machine = struct.unpack('<H' if header[5] == 1 else '>H', header[18:20])[0]
        if expected is None or machine != expected:
            print(f'INCOMPATIBLE: {library.relative_to(root)} (ELF machine={machine}, host={expected})')
            mismatches += 1
    print(f'Architecture mismatches/unknowns: {mismatches}')
    print('This checks ELF architecture only; matching CPU does not prove Python ABI or runtime compatibility.')
    return 1 if mismatches else 0


if __name__ == '__main__':
    sys.exit(main())
