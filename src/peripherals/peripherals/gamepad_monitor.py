#!/usr/bin/env python3
"""ArmPi signal monitor with optional live jog parameter editing; no motion publisher."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import sys
import time

from peripherals.gamepad_monitor_data import EventRecorder, JoystickReader, servo_command_data
from peripherals.jog_settings import JogParameterClient, parse_joint_settings

TOPICS = {
    'command': '/servo_controller',
    'driver': '/ros_robot_controller/bus_servo/set_position',
    'state': '/controller_manager/servo_states',
}
LABELS = {'input': '手柄输入', 'command': '上层目标', 'driver': '底层指令',
          'state': '命令缓存', 'device': '设备', 'session': '记录', 'settings': '点动参数'}


def create_monitor_node(emit):
    # Delayed imports allow --help, data tests and GUI tests without ROS startup.
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from servo_controller_msgs.msg import ServosPosition, ServoStateList
    from ros_robot_controller_msgs.msg import ServosPosition as DriverPositions

    class MonitorNode(Node):
        def __init__(self):
            super().__init__('gamepad_signal_monitor', enable_rosout=False,
                             start_parameter_services=False)
            self.topics = {source: self.resolve_topic_name(topic) for source, topic in TOPICS.items()}
            qos = QoSProfile(depth=100)
            self.create_subscription(ServosPosition, TOPICS['command'],
                                     self.command, qos)
            self.create_subscription(DriverPositions, TOPICS['driver'],
                                     self.driver, qos)
            self.create_subscription(ServoStateList, TOPICS['state'],
                                     self.state, qos)

        def command(self, msg):
            emit('command', 'positions', servo_command_data(msg, self.topics['command']))

        def driver(self, msg):
            emit('driver', 'positions', servo_command_data(msg, self.topics['driver'], 'pulse'))

        def state(self, msg):
            emit('state', 'cache', {
                'topic': self.topics['state'],
                'positions': [{'id': int(item.id), 'position': int(item.position)}
                              for item in msg.servo_state],
            })

    return MonitorNode()


class MonitorWindow:
    def __init__(self, root, device, log_dir, record=True):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk, self.root = tk, ttk, root
        self.node = None
        self.executor = None
        self.control = None
        self.closed = False
        self.seq = 0
        self.started = time.monotonic()
        self.last_received = {}
        self.counts = dict.fromkeys(TOPICS, 0)
        self.input_changes = 0
        self.buttons = {}
        self.axes = {}
        self.servo_rows = {}
        self.reader = JoystickReader(device, self.emit)
        self.recorder = EventRecorder(log_dir)
        self.next_graph = 0.0
        self.record_error = ''
        self.paused = tk.BooleanVar(value=False)
        self.show_states = tk.BooleanVar(value=False)
        self.record_text = tk.StringVar(value='未记录')
        self.input_text = tk.StringVar(value='等待设备；初始化快照不计为新输入')
        self.graph_text = tk.StringVar(value='等待 ROS 话题')
        self.step_text = tk.StringVar(value='5')
        self.duration_text = tk.StringVar(value='0.10')
        self.jog_current = tk.StringVar(value='节点当前值：未读取（输入框是待应用值）')
        self.jog_status = tk.StringVar(value='先读取当前值，再修改并应用；仅影响后续关节点动，重启后恢复启动参数。')
        self._build()
        root.protocol('WM_DELETE_WINDOW', self.close)
        if record:
            self.toggle_recording()

    def _build(self):
        tk, ttk = self.tk, self.ttk
        self.root.title('ArmPi · 遥控输入与控制输出监视器')
        self.root.geometry('1180x820')
        self.root.minsize(920, 650)
        style = ttk.Style(self.root)
        style.configure('Title.TLabel', font=('sans-serif', 17, 'bold'))
        style.configure('Treeview', rowheight=26)
        body = ttk.Frame(self.root, padding=14)
        body.pack(fill='both', expand=True)
        ttk.Label(body, text='遥控输入 → 舵机目标 → 底层指令', style='Title.TLabel').pack(anchor='w')
        ttk.Label(body, text='输入/输出监视 + 关节点动参数调节 · 不直接发送运动命令 · 输出不代表实际到位').pack(anchor='w', pady=(4, 10))
        toolbar = ttk.Frame(body)
        toolbar.pack(fill='x')
        self.record_button = ttk.Button(toolbar, text='开始记录', command=self.toggle_recording)
        self.record_button.pack(side='left')
        ttk.Button(toolbar, text='清空事件列表', command=self.clear_events).pack(side='left', padx=8)
        ttk.Checkbutton(toolbar, text='暂停事件列表（仍记录）', variable=self.paused).pack(side='left')
        ttk.Checkbutton(toolbar, text='列表显示命令缓存', variable=self.show_states).pack(side='left', padx=8)
        ttk.Label(body, textvariable=self.record_text, wraplength=1120).pack(anchor='w', pady=6)
        ttk.Label(body, textvariable=self.graph_text, wraplength=1120).pack(anchor='w', pady=(0, 8))
        settings = ttk.LabelFrame(body, text='关节点动参数 · 应用于正在运行的手柄节点', padding=8)
        settings.pack(fill='x', pady=(0, 8))
        row = ttk.Frame(settings)
        row.pack(fill='x')
        ttk.Label(row, text='joint_step (pulse)').pack(side='left')
        step_entry = ttk.Spinbox(row, from_=1, to=20, increment=1, width=6, textvariable=self.step_text)
        step_entry.pack(side='left', padx=(5, 14))
        ttk.Label(row, text='joint_duration (秒)').pack(side='left')
        duration_entry = ttk.Spinbox(row, from_=0.02, to=30, increment=0.01, width=8,
                                     textvariable=self.duration_text)
        duration_entry.pack(side='left', padx=(5, 14))
        read_button = ttk.Button(row, text='读取当前值', command=lambda: self.request_jog('read'))
        read_button.pack(side='left', padx=4)
        apply_button = ttk.Button(row, text='应用两项参数', command=lambda: self.request_jog('apply'))
        apply_button.pack(side='left', padx=4)
        self.jog_widgets = (step_entry, duration_entry, read_button, apply_button)
        ttk.Label(settings, textvariable=self.jog_current).pack(anchor='w', pady=(5, 0))
        ttk.Label(settings, textvariable=self.jog_status, wraplength=1080).pack(anchor='w')
        top = ttk.Panedwindow(body, orient='horizontal')
        top.pack(fill='both', expand=False)
        inputs = ttk.LabelFrame(top, text='① Linux 手柄输入（原始编号）', padding=8)
        outputs = ttk.LabelFrame(top, text='② / ③ 最近舵机目标与命令缓存', padding=8)
        top.add(inputs, weight=1)
        top.add(outputs, weight=2)
        ttk.Label(inputs, textvariable=self.input_text, wraplength=360).pack(anchor='w')
        self.button_frame = ttk.Frame(inputs)
        self.button_frame.pack(fill='x', pady=8)
        self.axis_frame = ttk.Frame(inputs)
        self.axis_frame.pack(fill='x')
        self.servo_table = self._table(outputs,
            ('id', 'target', 'driver', 'state', 'duration'),
            ('舵机 ID', '上层目标 / 单位', '底层 pulse', '缓存 pulse', '时长 上层/底层(s)'),
            (65, 145, 100, 100, 160), height=8)
        ttk.Label(outputs, text='缓存是上层已发位置记录，并非编码器实测。\n底层指令是串口驱动的输入，不能证明串口已发送或舵机已执行。',
                  wraplength=650).pack(anchor='w', pady=8)
        events = ttk.LabelFrame(body, text='事件时间线 · 最近 500 条（完整接收记录见文件）', padding=6)
        events.pack(fill='both', expand=True, pady=(12, 0))
        self.events = self._table(events, ('time', 'source', 'details'),
                                 ('本机接收时间', '来源', '内容'), (150, 100, 780), height=10)
        self.events.tag_configure('input', foreground='#17653a')
        self.events.tag_configure('command', foreground='#1553a0')
        self.events.tag_configure('driver', foreground='#8b4500')

    def _table(self, parent, columns, headings, widths, height):
        frame = self.ttk.Frame(parent)
        frame.pack(fill='both', expand=True)
        table = self.ttk.Treeview(frame, columns=columns, show='headings', height=height)
        for column, title, width in zip(columns, headings, widths):
            table.heading(column, text=title)
            table.column(column, width=width, minwidth=45)
        scrollbar = self.ttk.Scrollbar(frame, orient='vertical', command=table.yview)
        table.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side='right', fill='y')
        table.pack(fill='both', expand=True)
        return table

    def request_jog(self, action):
        if self.control is None:
            self.jog_status.set('参数连接尚未建立')
            return
        try:
            values = parse_joint_settings(self.step_text.get(), self.duration_text.get()) if action == 'apply' else None
        except ValueError as exc:
            self.jog_status.set(f'未发送：{exc}')
            return
        if self.control.request(action, values):
            self.jog_status.set(f'等待 {self.control.target} 响应…')
            for widget in self.jog_widgets:
                widget.configure(state='disabled')
            self.emit('settings', 'request', {'action': action, 'target': self.control.target,
                                             'values': values})

    def on_jog_result(self, action, successful, values, message):
        for widget in self.jog_widgets:
            widget.configure(state='normal')
        self.jog_status.set(message)
        if successful:
            self.jog_current.set(f'上次确认 {datetime.now():%H:%M:%S}：joint_step={values["joint_step"]} pulse，'
                                 f'joint_duration={values["joint_duration"]:g} 秒')
            self.step_text.set(str(values['joint_step']))
            self.duration_text.set(str(values['joint_duration']))
        else:
            self.jog_current.set('节点当前值：本次未确认，请重新读取')
        self.emit('settings', action, {'successful': successful, 'values': values,
                  'target': self.control.target if self.control else None, 'message': message})

    def toggle_recording(self):
        if self.recorder.active:
            self.emit('session', 'stop', {'message': '用户停止记录'})
            self.recorder.close()
        else:
            try:
                self.recorder.start()
                self.record_error = ''
                self.emit('session', 'start', {
                    'schema_version': 1, 'device': self.reader.device, 'topics': TOPICS,
                    'time_basis': 'local receive wall clock and monotonic elapsed; no causal pairing',
                    'state_semantics': 'command cache, not measured servo feedback',
                })
            except OSError as exc:
                self.record_error = str(exc)
        self._record_status()

    def _record_status(self):
        self.record_button.configure(text='停止记录' if self.recorder.active else '开始新记录')
        if self.record_error:
            text = f'记录失败：{self.record_error}（监视仍继续，请更换可写目录后重启）'
        elif self.recorder.path:
            text = f'{"正在记录" if self.recorder.active else "已停止"} · {self.recorder.rows} 条 · {self.recorder.path}（同时保存 CSV）'
        else:
            text = '未记录；点击“开始新记录”保存 JSONL 和 CSV'
        self.record_text.set(text)

    def emit(self, source, kind, data):
        self.seq += 1
        now = time.monotonic()
        event = {'seq': self.seq, 'time': datetime.now().astimezone().isoformat(timespec='milliseconds'),
                 'elapsed_s': round(now - self.started, 6),
                 'source': source, 'kind': kind, 'data': data}
        try:
            self.recorder.write(event)
        except OSError as exc:
            self.record_error = str(exc)
            self._record_status()
        self.last_received[source] = now
        if source in self.counts:
            self.counts[source] += 1
        if source == 'input':
            self._input(data)
            prefix = '初始快照' if data['initial'] else '变化'
            detail = f'{prefix} {kind}[{data["index"]}] = {data["value"]}'
        elif source in TOPICS:
            self._servos(source, data)
            detail = ', '.join(f'ID {p["id"]}: {p["position"]:g}' for p in data['positions'])
            if source != 'state':
                detail += f' | {data["unit"]} | {data["duration_s"]:g}s'
        else:
            detail = data.get('message', json.dumps(data, ensure_ascii=False))
            if source == 'device':
                self.input_text.set(detail)
                for label in self.buttons.values():
                    label.configure(text='?', background='#dddddd')
                for label, bar in self.axes.values():
                    label.configure(text='未知')
                    bar['value'] = 0
        if not self.paused.get() and (source != 'state' or self.show_states.get()):
            self.events.insert('', 'end', values=(event['time'][11:], LABELS[source], detail), tags=(source,))
            children = self.events.get_children()
            if len(children) > 500:
                self.events.delete(*children[:len(children) - 500])
            self.events.yview_moveto(1)

    def _input(self, data):
        index = data['index']
        if not data['initial']:
            self.input_changes += 1
        self.input_text.set(f'{self.reader.device} · 非初始化事件 {self.input_changes} 次')
        if data['kind'] == 'button':
            if index not in self.buttons:
                cell = self.ttk.Frame(self.button_frame)
                cell.grid(row=index // 8, column=index % 8, padx=2, pady=2)
                self.ttk.Label(cell, text=f'B{index}').pack()
                self.buttons[index] = self.tk.Label(cell, width=4)
                self.buttons[index].pack()
            self.buttons[index].configure(text='按下' if data['value'] else '松开',
                background='#9de0b5' if data['value'] else '#e7e9ed')
        elif data['kind'] == 'axis':
            if index not in self.axes:
                row = self.ttk.Frame(self.axis_frame)
                row.pack(fill='x', pady=2)
                label = self.ttk.Label(row, width=23)
                label.pack(side='left')
                bar = self.ttk.Progressbar(row, maximum=2, length=120)
                bar.pack(side='left', fill='x', expand=True)
                self.axes[index] = label, bar
            label, bar = self.axes[index]
            label.configure(text=f'A{index}: {data["value"]:6d} ({data["normalized"]:+.2f})')
            bar['value'] = data['normalized'] + 1

    def _servos(self, source, data):
        for position in data['positions']:
            sid = position['id']
            row = self.servo_rows.setdefault(sid, {'command': '—', 'driver': '—', 'state': '—',
                                                  'command_duration': '—', 'driver_duration': '—'})
            row[source] = f'{position["position"]:g}'
            if source == 'command':
                row[source] += f' {data["unit"]}'
            if source != 'state':
                row[source + '_duration'] = f'{data["duration_s"]:g}'
            values = (sid, row['command'], row['driver'], row['state'],
                      f'{row["command_duration"]} / {row["driver_duration"]}')
            if self.servo_table.exists(str(sid)):
                self.servo_table.item(str(sid), values=values)
            else:
                self.servo_table.insert('', 'end', iid=str(sid), values=values)

    def clear_events(self):
        children = self.events.get_children()
        if children:
            self.events.delete(*children)

    def tick(self):
        if self.closed:
            return
        self.reader.poll()
        if self.executor is not None:
            # Bound work per GUI frame; ROS and widgets run on the same thread.
            deadline = time.monotonic() + 0.008
            for _ in range(32):
                self.executor.spin_once(timeout_sec=0)
                if time.monotonic() >= deadline:
                    break
        if self.control is not None:
            self.control.poll()
        now = time.monotonic()
        if now >= self.next_graph:
            self.next_graph = now + 1
            parts = []
            for source, topic in TOPICS.items():
                publishers = self.node.count_publishers(self.node.topics[source]) if self.node else 0
                age = f'{now - self.last_received[source]:.1f}s 前' if source in self.last_received else '未收到'
                parts.append(f'{LABELS[source]}: 发布者 {publishers} / 接收 {self.counts[source]} / {age}')
            self.graph_text.set('    |    '.join(parts))
            self._record_status()
        self.root.after(20, self.tick)

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.reader.close()
        if self.control is not None:
            self.control.close()
        if self.recorder.active:
            self.emit('session', 'stop', {'message': '窗口关闭'})
        self.recorder.close()
        self.root.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default='/dev/input/js0', help='Linux joystick path')
    parser.add_argument('--control-node', default='/joystick_control', help='Target node for joint jog parameters')
    parser.add_argument('--log-dir', default=str(Path(os.environ.get('ARMPI_WS', Path.home())) / 'log/gamepad_monitor'))
    parser.add_argument('--no-record', action='store_true', help='Start without automatic recording')
    options, ros_args = parser.parse_known_args(args)
    if ros_args and ros_args[0] != '--ros-args':
        parser.error('未知参数: ' + ' '.join(ros_args))
    try:
        import tkinter as tk
        root = tk.Tk()
    except ImportError:
        print('缺少 Tk：请运行 sudo apt install python3-tk', file=sys.stderr)
        return 1
    except tk.TclError as exc:
        print(f'无法打开图形窗口，请在本机桌面终端运行：{exc}', file=sys.stderr)
        return 1
    app = None
    node = executor = None
    initialized = False
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        app = MonitorWindow(root, options.device, options.log_dir, not options.no_record)
        rclpy.init(args=ros_args)
        initialized = True
        node = create_monitor_node(app.emit)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        app.node, app.executor = node, executor
        app.control = JogParameterClient(node, options.control_node, app.on_jog_result)
        app.jog_status.set(f'目标：{app.control.target}；点击读取当前值，再修改并应用。')
        app.emit('session', 'ros_ready', {'topics': node.topics})
        app.tick()
        root.mainloop()
    except KeyboardInterrupt:
        pass
    except (ImportError, RuntimeError) as exc:
        print(f'监视器启动/运行失败，请先 source scripts/setup_pc.bash：{exc}', file=sys.stderr)
        return 1
    finally:
        if app is not None:
            app.close()
        else:
            root.destroy()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
