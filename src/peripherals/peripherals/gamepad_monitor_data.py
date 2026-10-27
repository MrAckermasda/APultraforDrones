"""Read-only Linux joystick input and portable event recording (no ROS imports)."""
import csv
from datetime import datetime
import errno
import json
import os
from pathlib import Path
import struct
import time


def decode_joystick_event(packet):
    """JS_EVENT_INIT is a state snapshot, never a fresh button press."""
    if len(packet) != 8:
        raise ValueError('Linux joystick event must contain 8 bytes')
    device_ms, value, flags, index = struct.unpack('<IhBB', packet)
    kind = {1: 'button', 2: 'axis'}.get(flags & 0x7f, 'unknown')
    return {
        'kind': kind, 'index': index, 'value': value,
        'normalized': max(-1.0, min(1.0, value / 32767.0)) if kind == 'axis' else None,
        'initial': bool(flags & 0x80), 'device_ms': device_ms,
    }


class JoystickReader:
    """Nonblocking independent reader; never grabs the input device exclusively."""
    def __init__(self, device, emit):
        self.device = str(device)
        self.emit = emit
        self.fd = None
        self.retry_at = 0.0
        self.status = '尚未连接'
        self._reported_status = None

    def _status(self, text):
        self.status = text
        if text != self._reported_status:
            self._reported_status = text
            self.emit('device', 'status', {'device': self.device, 'message': text})

    def poll(self, limit=128):
        if self.fd is None:
            if time.monotonic() < self.retry_at:
                return
            self.retry_at = time.monotonic() + 1.0
            try:
                self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                self._status(f'无法读取 {self.device}: {exc.strerror}')
                return
            self._status(f'已连接 {self.device}')
        try:
            for _ in range(limit):
                packet = os.read(self.fd, 8)
                if len(packet) != 8:
                    raise OSError(errno.ENODEV, '设备断开或事件不完整')
                data = decode_joystick_event(packet)
                self.emit('input', data['kind'], data)
        except BlockingIOError:
            pass
        except OSError as exc:
            self.close()
            self.retry_at = time.monotonic() + 1.0
            self._status(f'连接断开: {exc.strerror}')

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class EventRecorder:
    """One JSONL and one CSV per recording; errors are surfaced to the UI."""
    def __init__(self, directory):
        self.directory = Path(directory).expanduser()
        self.json_file = None
        self.csv_file = None
        self.path = None
        self.rows = 0

    @property
    def active(self):
        return self.json_file is not None

    def start(self):
        if self.active:
            return
        self.directory.mkdir(parents=True, exist_ok=True)
        name = datetime.now().strftime('gamepad_%Y%m%d_%H%M%S_%f')
        self.path = self.directory / (name + '.jsonl')
        try:
            self.json_file = self.path.open('x', encoding='utf-8', buffering=1)
            self.csv_file = self.path.with_suffix('.csv').open(
                'x', encoding='utf-8-sig', newline='', buffering=1)
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(['seq', 'time', 'elapsed_s', 'source', 'kind', 'data_json'])
            self.csv_file.flush()
            self.rows = 0
        except OSError:
            self.close()
            raise

    def write(self, event):
        if not self.active:
            return
        try:
            self.json_file.write(json.dumps(event, ensure_ascii=False) + '\n')
            self.writer.writerow([event[key] for key in ('seq', 'time', 'elapsed_s', 'source', 'kind')]
                                 + [json.dumps(event['data'], ensure_ascii=False)])
            self.csv_file.flush()
            self.rows += 1
        except OSError:
            self.close()
            raise

    def close(self):
        handles = (self.json_file, self.csv_file)
        self.json_file = self.csv_file = None
        for handle in handles:
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass


def servo_command_data(msg, topic, unit=None):
    return {
        'topic': topic, 'duration_s': float(msg.duration),
        'unit': msg.position_unit if unit is None else unit,
        'positions': [{'id': int(item.id), 'position': float(item.position)}
                      for item in msg.position],
    }
