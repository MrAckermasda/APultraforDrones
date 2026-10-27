# 电脑 USB 手柄控制：排障与启动

日期：2026-10-27。适用于接收器已从控制板移到电脑的连接方式。环境：x86_64、ROS 2 Jazzy、Python 3.12.3；FK/IK 原 ARM 扩展仍未适配，因此先使用单关节点动。

## 本次观察和原因

- Linux 已枚举 `/dev/input/js0`，设备 VID/PID 为 `2563:0575`。pygame 实际读取到 `USB WirelessGamepad`，4 个轴、13 个按键、1 个 hat；当前进程可访问输入设备。
- `/proc/bus/input/devices` 的设备名带末尾空格，pygame 返回的名称没有空格；此次仍统一用 `.strip()` 匹配，避免其他环境名称差异，但不把末尾空格认定为本次故障原因。
- 18:43 的 launch 日志表明用户只启动了 `peripherals/joystick_control.launch.py`，该入口原本只包含 `joy_node` 与手柄节点。旧底层控制进程在此前已经退出。检查时用户已结束手柄进程，当前可见 ROS 图为空；这是检查时的快照，不等于按键测试时所有节点状态都已知。
- 原 `JoystickController.__init__()` 在初始化手柄前无条件等待 `kinematics/get_current_pose`，即使 mode=0 也会卡在那里。原 `update_buttons()` 还有 `while True`，进入后不返回，阻塞同一默认回调组的舵机状态订阅。
- 原 `joy_node` 发布的 `/joy` 不被本手柄程序订阅。本程序直接使用 pygame，多启动一个 joy_node 无法补齐驱动/服务。
- 串口 `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` 存在，`root:dialout`、0660；账号 lyl 已属于 dialout，但此次检查进程尚无该组身份，`accessible=False`。实际操作终端应单独核验。

## 已修复内容

1. `enable_kinematics` 默认 false；单关节模式仅等待 `/controller_manager/init_finish`，缺服务时每 2 秒输出明确提示。启用此参数时也不在构造函数里无限等待 FK/IK，切换笛卡尔模式前检查服务可用性。
2. 手柄输入每 0.1 秒处理一批事件后返回；连接/断连检查每 0.5 秒执行一次，输入与连接管理放在同一回调组。移除原来占用回调的无限循环及后台 SDL 连接线程。
3. 只处理当前手柄实例；释放不新增动作、断连清除缓存、重连清空旧事件；不对未知手柄猜测按键映射。按键越界或其他实例的事件被忽略。
4. 十字键只在自身方向变化时点动，其他按键/模拟轴事件不会重复发送仍按住的方向。**不自动长按重复**，默认轴摇杆不新增映射；先用十字键/肩键/面板键短按。断连及松开只停止新增目标，不是硬件刹停或卸力。
5. 舵机状态按 ID 6、5、4、3、2、1 取值，不依赖消息数组顺序；缺 ID 时不接受点动。该状态来自上层命令缓存，不是持续读取的真实位置。
6. FK/IK 请求增加每项 1 秒等待上限；服务丢失/无响应时不发出该次 IK 运动。此功能没有修复原 ARM 库。
7. 手柄 launch 移除不被使用的 `joy_node`；保留原步长、指令时间、初始化姿态、START 动作和按键方向。

## pygame API 崩溃修复（2026-10-27）

用户日志已出现 `Joint control ready` 和 `Gamepad connected: 'USB WirelessGamepad'`，随后在 `update_buttons()` 调用 `self.js.get_attached()` 时退出。本机 pygame 2.5.2 的 `pygame.joystick.Joystick` 没有该方法；这次退出与 AVX2 编译警告无关，无需为此重编译 pygame。

已修改 `src/peripherals/peripherals/joystick_control.py`：连接检查使用 `get_init()`；物理拔出根据当前实例的 `JOYDEVICEREMOVED` 事件识别，并捕获 pygame 输入错误后清理连接。`get_init()` 仅表示对象已初始化，不能单独证明设备仍插着。速度参数、按键映射、初始化姿态和 START 行为保持原样。

`tests/test_pc_gamepad.py` 使用受限的 `spec_set` 模拟对象，避免 Mock 自动接受不存在的方法；补充缺失 API、其他设备拔出、输入读取错误的回归覆盖。

此修复已定向构建。保留正在工作的底层驱动和 servo_controller，在原手柄终端按 Ctrl+C 结束残留 launch，再按本文第三步启动手柄节点。启动会再次发送原初始化姿态（1 秒），先清空活动范围、松开所有按键；姿态完成后短按一次测试。日志证明程序已进入发送初始化命令的路径，不证明舵机已经执行成功。

## 第一步：仅确认手柄输入，不启动机械臂

```bash
cd /home/lyl/ros2_ws
python3 scripts/check_gamepad.py --seconds 15
```

在 15 秒内按下、松开面板键和十字键，应看到 `button[n]=...` 或 `axis[n]=...`。脚本仅只读打开 Linux 手柄设备，不导入 ROS、不连接控制板串口、不发送舵机命令。初始化状态不计为新按键事件。

若显示设备存在但 `Non-initial events: 0`，先检查手柄开机、配对与电量，并在观察窗口内操作；单凭接收器枚举不能确认无线输入正常。本次只运行了 1 秒观察，没有看到非初始化事件，因此尚未替用户验证实际按键输入。

## 启动前补齐 control_msgs（本次新增定位）

2026-10-27 18:59 的 sdk 启动日志显示：底层 ros_robot_controller 正常启动，但 servo_controller 启动即以退出码 1 结束。随后只导入 `servo_controller.controller_manager`（未构造节点），复现：

```text
ModuleNotFoundError: No module named 'control_msgs'
```

该依赖由 `joint_trajectory_action_controller.py` 导入 `FollowJointTrajectory` 时使用，启动阶段就需要；即使只做手柄关节点动也不能省略。`servo_controller/package.xml` 已声明此依赖，但构建包不会自动完成系统依赖安装。

当前检查 rclpy、std_srvs、sensor_msgs、trajectory_msgs、自定义消息、sdk 和 numpy 均能被 Python 找到，缺少的是 control_msgs。APT 模拟安装确认 `ros-jazzy-control-msgs` 可用，计划仅新增该包。实际执行非交互安装时 sudo 返回 `a password is required`，因此**本次尚未安装成功**，需要用户在本机终端输入自己的 sudo 密码执行：

```bash
sudo apt-get install -y ros-jazzy-control-msgs
```

若 APT 提示索引过期或下载 404，先 `sudo apt-get update` 再执行安装。安装后加载环境，运行只导入验证：

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
python3 -c 'from control_msgs.action import FollowJointTrajectory; import servo_controller.controller_manager; print("servo_controller imports OK")'
```

该检查不创建节点、不打开串口、不发送运动指令。只有输出 `servo_controller imports OK` 才能确认导入问题已解决；不需要为安装这个系统消息包重新构建本工作区。

如果原 sdk launch 中的底层驱动仍正常运行、只有 servo_controller 已退出，则保留它，另开一个终端补启动：

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
ros2 launch servo_controller servo_controller.launch.py enable_grasp:=false
```

不要同时重复启动 sdk 或第二套底层驱动。此终端保持运行，再按第三步启动手柄；手柄节点在控制器就绪后会发送原初始化姿态，因此启动前清空活动空间。

## 第二步：终端 A 启动完整的关节控制链

只运行一套底层驱动。以下命令适合旧驱动已退出的情况；如果你已有工作中的驱动，先检查，不要再开一套争用串口。

账号已在 dialout 组但终端未刷新的情况下，先执行：

```bash
newgrp dialout
```

这会进入一个刷新组身份的新 shell，随后在这个 shell 中运行：

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
python3 -c 'import os; print(os.access(os.environ["ARMPI_SERIAL_PORT"], os.R_OK | os.W_OK))'
```

权限检查应输出 True。若仍为 False，先处理该终端的设备路径/权限；此处不需要重复把账号加入 dialout 或改成所有人可写。

确认机械臂活动空间已清空、控制板及舵机供电正常后启动：

```bash
ros2 launch sdk armpi_ultra.launch.py enable_kinematics:=false
```

这个已有入口会启动 `ros_robot_controller` 和 `servo_controller`，同时关闭依赖运动学的 grasp 与运动学节点。保持终端 A 运行，检查是否有串口报错。不要再单独启动第二个 `ros_robot_controller`。

如果现有 `ros_robot_controller` 已稳定运行且你要保留它，则只补启动 `servo_controller`，用这一条替代上面的 sdk 启动方式：

```bash
ros2 launch servo_controller servo_controller.launch.py enable_grasp:=false
```

这两个方案是替代关系，不能叠加启动重复的节点。

## 第三步：终端 B 启动电脑端手柄程序

**这个节点仍保留原来的启动初始化姿态命令（1 秒）。即使没有按键也可能在启动时运动；START 还会执行回中。点动调速参数不会使这两种动作随之变慢。先松开所有按键、清空运动范围并准备可靠的停止方式，再启动。**

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
ros2 launch peripherals joystick_control.launch.py \
  enable_kinematics:=false joint_step:=5 joint_duration:=0.10
```

日志应依次出现已加载的调速参数、`Joint control ready` 以及 `Gamepad connected: 'USB WirelessGamepad'`。若停在 `Waiting for /controller_manager/init_finish`，先看终端 A，手柄尚未进入输入处理阶段。

启动姿态完成后，先短按一次已知按键：

| 按键 | 单关节模式作用 |
|---|---|
| 十字键上/下 | ID 5 正/负方向点动 |
| 十字键右/左 | ID 6 负/正方向点动 |
| L1/L2 | ID 1 负/正方向点动（夹爪） |
| R1/R2 | ID 3 正/负方向点动 |
| 方块/圆圈 | ID 2 正/负方向点动 |
| 叉/三角 | ID 4 负/正方向点动 |

`joint_step=5` 是舵机位置单位增量，不是 5 度。`joint_duration=0.10` 是单条点动指令时间，不代表保证固定速度。默认 `enable_kinematics=false` 时模式键会提示保留关节模式，不能切入笛卡尔控制。尚未完成 FK/IK 适配时不要将它改成 true 来尝试绕过限制。

需要再次短按时先松开。松开和 Ctrl+C 都不能保证已经发出的运动立即停止。

## 仍无反应时如何定位

先看两个终端输出，再用下面的只读 ROS 图检查：

```bash
source /home/lyl/ros2_ws/scripts/setup_pc.bash
ros2 node list --no-daemon
ros2 topic info /servo_controller --verbose --no-daemon
ros2 topic info /controller_manager/servo_states --verbose --no-daemon
```

正常链路应包含 `ros_robot_controller`、`controller_manager`、`servo_manager`、`joystick_control`。`/servo_controller` 应有手柄发布者和上层控制器订阅者，状态话题应有发布者和手柄订阅者。话题端点存在不证明已经发出了命令或硬件已执行。

- `Waiting for /controller_manager/init_finish`：缺上层舵机管理器，或其自身在等底层驱动。
- `Servo state not ready yet`：尚未收到完整 ID 状态；不能靠随意补一个 500 位置绕过。
- 没有 `Gamepad connected`：检查 USB 枚举、pygame 可访问性、手柄名称。
- 有 connected 但只读脚本无按键变化：接收器可见不代表手柄无线配对成功。
- 输入和 ROS 链路均正常但舵机不动：再查控制板供电、串口错误和硬件响应；本次未执行这一步实机验证。

## 验证和回退

- 原有测试与 `tests/test_pc_gamepad.py` 合计 **149 passed**（含此次 pygame API 回归覆盖）。覆盖无 IK 服务时的构造路径、回调返回、释放/断连不新增目标、设备过滤、按 ID 状态映射、模式切换条件、服务超时、launch 参数等。
- `colcon build --base-paths src/peripherals --packages-select peripherals --symlink-install` 成功。
- `ros2 launch peripherals joystick_control.launch.py --show-args` 确认默认 `enable_kinematics=false`；未启动实际控制节点。
- pygame 枚举与只读 Linux 手柄读取成功；未确认用户按键事件、串口通信、初始化动作或点动效果。
- 新增脚本 `scripts/check_gamepad.py`；新增本文；更新两份操作说明和 PC 迁移修改记录。
- 备份：`.migration-backup/2026-10-27-pc-gamepad-185019/`。需要回退代码时，先安全停止控制流程，将备份对应源码/launch 恢复后重新构建；回退也会恢复无条件等待 FK/IK 和旧无限循环问题。

本次 pygame API 修复补充验证：真实 pygame 成功连接当前 `USB WirelessGamepad` 并运行连接和输入轮询回调，舵机发布函数被 mock，没有发送运动命令。未验证真实按键点动、初始化动作完成或机械臂运动。AVX2 和 `pkg_resources` 警告仍存在，未阻止测试通过。

复现离线验证时必须先加载工作区环境，否则可能因找不到自定义消息 `ros_robot_controller_msgs` 在收集测试时失败：

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
python3 -m pytest -q tests/test_pc_gamepad.py tests/test_joystick_speed.py \
  tests/test_servo_read_timeout.py tests/test_pc_portability.py tests/test_software_paths.py
```

本次修复前备份：`.migration-backup/2026-10-27-pygame-api-191538/`。回退此次代码会恢复 `get_attached()` 崩溃；备份仅用于复盘与比较。上述 185019 备份属于更早的手柄适配阶段。
