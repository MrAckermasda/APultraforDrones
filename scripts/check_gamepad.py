#!/usr/bin/env python3
"""Read Linux joystick events only; never imports ROS or opens a servo port."""
import argparse
import os
import select
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/input/js0')
    parser.add_argument('--seconds', type=int, default=15)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error('--seconds must be between 1 and 300')
    try:
        fd = os.open(args.device, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as exc:
        parser.exit(1, f'Cannot read {args.device}: {exc}\n')
    count = 0
    print(f'Reading {args.device} for {args.seconds}s. Press/release buttons or D-pad. '
          'No ROS/servo commands are sent.', flush=True)
    deadline = time.monotonic() + args.seconds
    try:
        while time.monotonic() < deadline:
            ready, _, _ = select.select([fd], [], [], max(0, deadline - time.monotonic()))
            if not ready:
                break
            data = os.read(fd, 8)
            if len(data) != 8:
                raise OSError('Device disconnected or incomplete joystick event')
            _, value, event_type, number = struct.unpack('<IhBB', data)
            if event_type & 0x80:
                continue  # Initial state does not prove a fresh wireless report.
            kind = {1: 'button', 2: 'axis'}.get(event_type, f'type={event_type}')
            print(f'{kind}[{number}]={value}', flush=True)
            count += 1
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        print(f'Input read failed: {exc}', flush=True)
        return 1
    finally:
        os.close(fd)
    print(f'Non-initial events: {count}. ' + (
        'Input changes received.' if count else
        'No input changes observed; device presence alone does not prove wireless pairing.'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
