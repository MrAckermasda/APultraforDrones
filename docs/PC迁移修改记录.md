# ArmPi Ultra 从树莓派迁移到电脑：修改记录

日期：2026-10-27（Asia/Shanghai）

更新：`transforms3d` 已安装，用户已加入 `dialout`；当前旧会话尚未刷新。下方原始记录保留首次适配状态，最新结果见文末“后续复查”。

## 当前结果

已完成连接配置、资源路径和启动参数的移植；在本机完成 17 个 ROS 包的构建及 11 项不连接硬件的回归测试。**尚未完成实机通信、运动控制或整套应用运行验证。** 原包的 ARM 运动学库与本机 x86_64 不兼容，是完整功能的明确阻塞项。

## 实测环境

| 项目 | 本机信息 |
|---|---|
| 系统 | Ubuntu 24.04 LTS |
| CPU | x86_64 |
| ROS | ROS 2 Jazzy |
| Python | 3.12.3 |
| pyserial / NumPy / PyYAML | 3.5 / 1.26.4 / 6.0.1 |
| setuptools | 68.1.2 |
| 工作区 | `/home/lyl/ros2_ws`，路径已支持迁移 |
| 当前枚举到的串口 | `/dev/ttyUSB0` |
| 对应 by-id | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` |
| USB 转换器 | CH340，VID:PID `1a86:7523`，驱动 `ch341` |
| 串口权限 | `root:dialout`、`0660`；当前 `lyl` 用户不在 `dialout` 中 |
| 原树莓派 ROS/Python、控制板固件 | 未提供，未确认 |

USB 枚举只证明串口转换器存在，尚未验证它连接的是这套控制板。`1000000` 来自原始 SDK 默认值，并非本次测得的固件参数。

## 修改内容

### 1. 可携带的本机配置

新增 [config/pc.env](../config/pc.env) 和 [scripts/setup_pc.bash](../scripts/setup_pc.bash)。启动脚本根据自身位置计算 `ARMPI_WS`，加载 ROS 基础环境和已构建的当前工作区，并加载配置。未修改 `.bashrc`。

```bash
cd /你的路径/ros2_ws
source scripts/setup_pc.bash
```

| 配置项 | 本次默认值 | 作用 |
|---|---|---|
| `ARMPI_SERIAL_PORT` | 当前 CH340 的 by-id 路径 | 控制板串口，移植时核实 |
| `ARMPI_BAUDRATE` | `1000000` | 控制板协议波特率 |
| `CHASSIS_TYPE` | `None` | 默认独立机械臂；按实物选择底盘 |
| `CAMERA_TYPE` | `usb_cam` | 默认 RGB 相机；`aurora` 为现有深度相机分支 |
| `ARMPI_VIDEO_DEVICE` | `/dev/video0` | 普通 USB 摄像头设备 |
| `MACHINE_TYPE` | `ArmPiUltra` | 原代码中型号分支的默认值 |
| `ASR_LANGUAGE` | `English` | 保留原语音默认语言，可覆盖 |
| `ARMPI_ROS_DISTRO` | 未设置时沿用当前 ROS，最后回退 `jazzy` | 指定 `/opt/ros` 中的发行版 |

这些硬件默认值是配置假设，尚未识别实物底盘和相机。切换到滑轨底座时务必设置 `CHASSIS_TYPE=Slide_Rails`，因为它影响运动学底座高度。

已有环境变量优先于 `pc.env` 的默认值。要改已有终端的值，显式 `export`，或 `unset` 对应变量再重新 source；只改默认值不会覆盖已经导出的变量。

### 2. 串口参数真正传到底层

- `ros_robot_controller_node.py` 新增 `serial_port`、`baudrate` ROS 参数，并记录连接配置。
- `ros_robot_controller.launch.py` 暴露同名参数；波特率强制解析为整数，串口解析为字符串。
- `Board` 支持环境变量，未指定时仍回退 `/dev/rrc` 和 `1000000`。
- `HwInterfaces` 将传入的串口和波特率交给 `Board`；修复以前参数看似可配、实际被忽略的问题。

优先级：显式 launch/ROS 参数（或 Python 构造参数）→ 环境变量 → 底层默认值。`pc.env` 通过设置环境变量参与这条链。

确认接线、权限和机械臂状态后，可单独启动底层节点：

```bash
source scripts/setup_pc.bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  baudrate:=1000000
```

此命令会真实打开串口，节点初始化也会写控制命令；本次没有执行。只查看参数可加 `--show-args`。

### 3. 去除旧主机绝对路径

- 批量整理 launch 文件，统一通过 `get_package_share_directory()` 查找已构建的包，移除 `need_compile` 分支及 `/home/ubuntu/ros2_ws` 路径。
- 补上部分原 launch 文件缺失的 `get_package_share_directory` 导入。
- 新增 `sdk/paths.py` 的 `workspace_path()`；运行时标定、模型、音频路径使用 `ARMPI_WS` 或源码位置定位。保留尾部 `/`，兼容原代码中的字符串拼接。
- `startup_check.py` 的音频播放改用参数列表，工作区路径含空格时也能正确传给播放器。
- 动作组目录通过 `ARMPI_ACTION_GROUP_PATH` 指定，默认 `~/software/armpi_ultra_control/ActionGroups`。
- HP60C 的配置目录通过 `ascamera_config_dir` launch 参数或 `ASCAMERA_CONFIG_DIR` 环境变量指定；需要实际厂商配置文件。
- YOLO 演示的外部模型目录通过 `ARMPI_YOLO_MODEL_DIR` 指定，默认当前用户的 `~/third_party_ros2/yolo/yolov8`。

**部署约束：目前保留源码工作区。** 标定程序会写回源码配置；仅复制 `install/` 不足以提供全部可写 YAML、音频和模型资源。纯安装目录部署需要后续单独整理数据安装规则与可写配置目录。

### 4. 启动与摄像头

- 环境变量读取补充默认值，避免原厂 shell 配置缺失时直接 `KeyError`。
- USB 摄像头增加 `video_device` launch 参数和 `ARMPI_VIDEO_DEVICE` 覆盖入口；保留原话题映射和像素格式。
- SDK 移除未使用的 `nav2_common` 导入，独立机械臂不会再无条件查找底盘包。
- 新增 `enable_kinematics` 参数，设为 `false` 时同时关闭运动学及依赖它的 grasp 节点；`servo_controller` 单独启动时也提供 `enable_grasp` 参数。

```bash
# 只查看参数，不连接硬件
ros2 launch peripherals usb_cam.launch.py --show-args
ros2 launch sdk armpi_ultra.launch.py --show-args

# 下面是真实启动命令，先完成实机准备
ros2 launch sdk armpi_ultra.launch.py enable_kinematics:=false
```

禁用运动学不会使 IK、自动抓取、分拣等功能恢复；它只用于隔离未适配的部分。整套 `bringup` 中的 `startup_check.py` 会发送初始姿态，不应当用作无动作的连通性检查。

### 5. udev 与依赖

- 两份 `99-ttyACM0.rules` 已统一，修正多余字符，按原控制板 `1a86:55d4` 匹配，使用 `dialout` / `0660`，创建 `/dev/rrc`。
- 本机 CH340 是 `1a86:7523`，不匹配上述原控制板规则；本次通过 by-id 配置使用它，没有把全部 USB 串口绑定成 `rrc`。
- 规则安装脚本根据自身目录查找规则，安装失败即停止，使用 `udevadm control --reload-rules`。没有执行安装脚本，也没有修改 `/etc/udev`。
- 补齐核心包及资源路径使用者的部分必要 `package.xml` 依赖，包括 `python3-transforms3d`；没有把原来的 `0.0.0` 冒充为实际固件或产品版本。
- 运动学 `setup.py` 增加 `.so` 打包规则，供今后放入兼容二进制后正常安装使用；这不改变现有二进制架构。

## 验证记录

| 检查 | 结果与范围 |
|---|---|
| 270 个 Python 文件语法解析 | 通过；原 `sdk/pid.py` 有无效转义的 SyntaxWarning，不影响本次解析 |
| 全部 `package.xml` 解析 | 通过 |
| 核心包构建 | 8 个包通过 |
| 全工作区 `colcon build --symlink-install` | 17 个包通过；未执行节点 |
| `tests/test_pc_portability.py` | 11 项通过，包括串口参数传递/优先级、模拟模式、迁移路径、launch 参数类型、相机设备覆盖、IK/grasp 关闭条件 |
| launch 参数检查 | 串口、摄像头、SDK 参数可解析；核心 include 路径可加载 |
| 两份 udev 规则语法 | `udevadm verify` 通过 |
| Bash 语法 | 配置和环境脚本通过 |
| 技能文件 | `quick_validate.py` 通过 |
| 核心 ELF 检查 | 返回 1，正确报告两个 ARM 库与本机不匹配 |
| 串口通信、摄像头图像、机械臂动作 | 未执行，未验证 |

系统缺少 `transforms3d`；尝试系统安装时 `sudo` 要求密码，因此未安装。测试使用 `apt-get download` 下载的 Ubuntu `python3-transforms3d 0.4.1+ds-1`，解包到 `/tmp/armpi-test-deps`，仅在测试进程临时添加 `PYTHONPATH`。测试通过不代表当前系统运行依赖已补齐。

构建记录见 `log/`。复查命令（系统依赖安装后）：

```bash
source scripts/setup_pc.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source scripts/setup_pc.bash
python3 -m pytest -q tests/test_pc_portability.py
python3 scripts/check_portability.py --core-only
```

最后一条在原 ARM 库替换前预期报告架构不兼容。去掉 `--core-only` 可额外扫描语音等可选二进制。该脚本不加载扩展，不打开串口。

## 尚未解决及下一步

1. **正／逆运动学库**：`src/driver/kinematics/kinematics/forward_kinematics.so`、`inverse_kinematics.so` 是 aarch64，本包中未发现其编译源码。需要厂商提供 x86_64、Python 3.12 匹配的版本，或提供源码后重新编译；不能靠重命名解决。
2. **语音扩展**：同样包含 aarch64 库，需要目标架构及 Python ABI 匹配的版本。现有语音功能未做运行验证。
3. **串口权限和系统依赖**：需要用户在终端完成管理员操作，之后注销并重新登录以刷新组成员身份：

   ```bash
   sudo apt-get install python3-transforms3d
   sudo usermod -aG dialout "$USER"
   ```

   然后重新打开终端，source 环境并运行只读检查。未更改用户组，也未尝试用 `chmod 777` 绕过权限。
4. **外部资源**：摄像头型号及其驱动、动作组、语音资源、深度处理与 YOLO/TensorRT 等应用依赖仍须按实际功能补齐。构建 Python 包不会检查所有运行时导入和模型文件。
5. **协议与实机验证**：确认 CH340 连接的是控制板、核实固件协议和供电接线后，再进行通信和受控动作验证。机械参数、舵机 ID、标定数据及消息协议保持原有定义。

## 复盘、回退与 Skill

修改前的既有文件保存在 `.migration-backup/2026-10-27/`，保留原目录结构；[修改文件列表](../.migration-backup/2026-10-27/changed-paths.txt) 用于逐文件比对。当前目录不是 Git 仓库，因此使用文件备份。只恢复需要回退的源码/配置，再重新构建；不要直接复用过期的 `build/` 和 `install/`。新增脚本、文档、测试和技能文件没有旧版本。

技能入口为根目录 [skill.md](../skill.md)，它链接到 [.agents/skills/armpi-porting/SKILL.md](../.agents/skills/armpi-porting/SKILL.md)。内容涵盖目标平台检查、串口与消息接口、路径策略、底盘/相机配置、二进制限制及分层验证。搬迁工作区时带上 `.agents` 隐藏目录；也可将 `armpi-porting` 文件夹复制到目标设备的 `~/.codex/skills/` 供后续会话发现。本文和技能都不将尚未完成的实机验证表述为已完成。


## 后续复查：依赖已安装，串口组等待会话刷新（2026-10-27）

- 系统 Python 成功导入 `transforms3d 0.4.1`。不再使用临时测试依赖，重新运行 `tests/test_pc_portability.py`，11 项全部通过。
- `id lyl` 和 `getent group dialout` 显示账号已加入 `dialout`（GID 20），但当前进程的 `id` / `os.getgroups()` 不含该组；这解释了用户报告的 `accessible=False`。
- 设备仍为 `root:dialout`、`0660`。使用 `sg dialout` 创建刷新组身份的子进程后，`os.access(..., R_OK | W_OK)` 返回 True。检查没有打开串口或发送指令。
- 用户可在操作终端运行 `newgrp dialout`，随后重新 `source scripts/setup_pc.bash` 并检查。该操作只影响新 shell 及其子进程，不会更新已运行的 IDE 或其他终端；注销桌面重新登录可整体刷新。无需重复执行 `usermod`。
- 用户完整扫描报告的 16 个架构不匹配文件包括 2 个正／逆运动学库和 14 个语音相关库/平台变体。CPU 架构问题仍未解决，与串口权限和依赖安装无关。

```bash
newgrp dialout
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
python3 scripts/check_portability.py --core-only
```

在新 shell 中串口应显示 `accessible=True`，两个运动学库仍会报告不兼容，检查脚本因此返回 1；此退出状态不表示串口权限修复失败。

## 新增 software 目录检查（2026-10-27）

已检查工作区 `software/`，详细结果见 [software包检查记录.md](software包检查记录.md)。找到原厂主控制界面及 24 个可读取的动作组，补充了之前未在 ROS 源码树中找到的应用资源。没有找到 FK/IK 实现源码或兼容的替代库。主 GUI 顶层直接导入运动学库，因此它的基本舵机/动作组功能也会被整个窗口启动失败间接阻塞。该目录的旧路径、shell 配置及独立 SDK 尚未迁移；本轮仅检查并记录，没有启动应用或改变其控制逻辑。

## software 路径修改与教程核对（2026-10-27）

已完成 software 中原厂工具的资源路径、Bash 启动环境、4 个桌面入口和活动数据集路径调整，并让两份独立 SDK 使用同一串口环境配置。详情及备份位置见 [software路径迁移与验证.md](software路径迁移与验证.md)。

已阅读用户提供的《3.机械臂高级控制》31 页教程，核对 DH 参数及 FK/IK 推导，指出第 18 页特定姿态化简与最终打印矩阵的问题。遥控入口位于 `src/peripherals/peripherals/joystick_control.py`；其单舵机模式仍无条件等待运动学服务，本次仅提供后续改造建议，没有直接启动或重写控制逻辑。参数表、源码入口、按键映射和两阶段建议见 [FK_IK资料核对与遥控适配建议.md](FK_IK资料核对与遥控适配建议.md)。


## 舵机读取请求等待修复（2026-10-27）

用户报告 get_state 请求发出后未见返回。源码确认 `bus_servo_read_and_unpack()` 使用无超时的队列读取，串口 timeout 对该等待无效；无回包时可占住服务回调。

已修改 ROS SDK 和服务回调：总线读取默认每项 1 秒期限，锁获取也有超时，丢弃请求前的旧队列数据，校验回包长度、ID 和命令，失败时服务 success=False 并输出字段日志；同时修正扭矩读取的方法名、接通已有电压读取实现。没有更改运动指令或协议编码。协议没有请求序号，迟到的同 ID/同命令回包仍无法完全区分；本次不声称解决所有链路问题。PWM 的独立读取路径不在本轮修复范围。

8 项新增模拟测试与已有 24 项共 32 项通过；未打开硬件连接进行测试。已核对新 Python 进程从源码加载修复，现有运行进程需用户停止并重新启动。日志曾出现位置响应 184、214、217，仅说明历史请求收到过响应，不作为当前位置或新修复实机验证结果。

备份：`.migration-backup/2026-10-27-servo-read-timeout/`。新增测试：`tests/test_servo_read_timeout.py`。重新读取步骤已加入首次运动指南第 9 节。


## 手柄点动调速参数化（2026-10-27）

用户已反馈手柄控制有效，希望调整控制后的机械臂运动速度。未指定增速或减速，因此默认动作保持原样，新增启动参数供选择；未通过启动节点进行实机调试。

- `joystick_control.py`：新增 `joint_step=10`、`joint_duration=0.05`、`cartesian_step=0.005`、`cartesian_duration=0.02`、`pitch_step=1.0`；接入原关节/笛卡尔回调，保留所有舵机 ID、按键方向、初始化姿态和 START 回中时间。
- 参数在运动发布器创建前进行有限值、类型及范围校验，作为只读启动参数；重启生效，不承诺在线修改。步长限制是配置保护，不是机械安全限位。
- launch 新增对应有类型的参数；`package.xml` 声明 `rcl_interfaces` 依赖。旧 `max_linear`、`max_angular` 和 `disable_servo_control` 不用于这个程序的机械臂调速/禁动，未借此声称支持新功能。
- 原 pygame 事件循环及长按重复语义不变。指令时间不是稳定命令周期，步长/时间不能直接等同整臂实测速度。此次没有解决运动学二进制兼容性，也没有重新校准机械参数。
- 新增 `tests/test_joystick_speed.py`，95 项离线用例与原有 32 项合计 **127 passed**；测试不打开硬件，保留原软件依赖警告。`peripherals` 已构建；已有进程需退出后重启才能加载新代码。
- 初次构建扫描到不相关 `software/ultralytics` 的包识别错误，`peripherals` 仍成功完成；后续构建限定 `--base-paths src/peripherals`，不改动 ultralytics 或系统依赖。

操作指南：[手柄速度调节说明.md](手柄速度调节说明.md)，并已在首次运动指南第 10 节加入入口与启动风险提示。参数优先级为启动显式值覆盖节点/launch 默认值，不自动从 `config/pc.env` 读取。

备份：`.migration-backup/2026-10-27-joystick-speed-173150/`。恢复原速度可不传新增覆盖参数；源码回退使用该目录对应原文件，并重新构建/加载 `peripherals`。实机速度与操作手感仍需用户在现有可工作配置上逐步短按验证。


## 手柄控制来源补充核查（2026-10-27）

用户指出仅启动底层节点时手柄似乎已有效。只读核查确认当前 launch 仅包含 `ros_robot_controller`；检查时进程及当前 ROS 图未发现 `joystick_control`，ROS 图仅有 `/ros_robot_controller`。底层 Joy 话题发布者 1、订阅者 0；总线舵机 set_position/set_state 话题发布者均为 0；未发现 `/dev/input/js*`。检查没有读取实际 Joy 数据，不把话题存在当作收到手柄消息的证据。

源码表明底层只把控制板上报的手柄数据发布出来，不执行按键到舵机的映射。若此状态下手柄仍能控制机械臂，更可能由控制板固件直接处理，但接收器连接位置和固件待核实。上次新增的点动参数只作用于 `peripherals/joystick_control`，不能承诺影响当前固件遥控速度；已在手柄速度说明开头纠正适用范围。未修改运动代码、未停止进程、未发送运动指令。下一步先确认手柄接收器连接位置，再确定可用的调速入口。

本次文档修改前备份：`.migration-backup/2026-10-27-joystick-route-181755`。


## STM32 工程与板载手柄控制核查（2026-10-27）

用户指出 STM32 资料目录。找到原始 RAR 与工作区 ArmPiUltra 解压目录；只读核对压缩包 1336 个条目、78 个应用/项目/库文件一致；Keil 69 个项目文件路径存在。发现 ps2_porting.o/.d 及 ps2_set_action_time 等手柄函数符号，但原包缺少其 ps2_porting.c/.h，当前项目和 main 未接入该模块；不能直接用此工程重建用户当前遥控固件。

配套原理图 USB-Host 使用 CH559T，通过 559_TX 连接核心板。需匹配实物版本，不能将“接收器插在 STM32 控制板上”简化为 USB 直接由 STM32 芯片处理。已经找到底层运动时间（毫秒）及串口 duration 传递路径，但没有找到当前遥控默认速度或对外调速协议。没有修改共用底层速度、启动节点、构建或烧录固件。

新资料中 LeArm.lib 提供 ARM fkine/ikine 库符号，并非 x86_64 Python 扩展或算法源码。详细证据、缺失文件与后续条件见 [STM32手柄源码核查.md](STM32手柄源码核查.md)，已在手柄调速说明顶部增加入口与范围纠正。

本次既有文档备份：`.migration-backup/2026-10-27-stm32-source-audit-183456`。


## 电脑 USB 手柄无响应修复（2026-10-27）

用户将接收器从控制板移到电脑后，启动 joystick_control 无响应。检查时 Linux 与 pygame 均识别手柄（2563:0575，USB WirelessGamepad，4 轴/13 键/1 hat，输入权限正常）。近期日志显示旧底层驱动已先退出，而手柄 launch 不包含底层和 servo_controller；检查时控制进程均已结束。源码另外确认构造函数无条件等待 FK/IK，以及输入回调无限循环占用默认回调组。

已改 joystick_control.py：enable_kinematics 默认 false，单关节模式只等待 controller_manager，缺依赖时输出提示；输入轮询单批返回，SDL 连接/断连管理移到同组定时回调；状态按舵机 ID 排序；过滤错误实例/索引、未知手柄、断连缓存。笛卡尔切换检查服务，FK/IK 请求有超时。launch 移除未被该程序订阅的 joy_node，新增 enable_kinematics 参数。保留初始化姿态、START 和点动默认值；十字键不再因无关事件重复点动，不自动长按重复。

新增 scripts/check_gamepad.py，仅只读 Linux 手柄设备；1 秒观察没有非初始化事件，不能认定无线配对已验证。串口设备存在但本次检查进程无 dialout 组身份，accessible=False，账号已属于 dialout；在指南提供 newgrp 与实际终端权限检查，没有修改系统权限或发送运动指令。

新增 tests/test_pc_gamepad.py 共 19 项，与原 127 项合计 146 passed；peripherals 定向构建成功，--show-args 确认参数。更新操作指南并新增 [电脑USB手柄控制排障与启动.md](电脑USB手柄控制排障与启动.md)。备份 `.migration-backup/2026-10-27-pc-gamepad-185019/`。无控制节点启动或实机运动验证；下一步用户在清空活动空间后按指南先验证输入，再启动控制链短按测试。


## control_msgs 系统依赖缺失定位（2026-10-27）

用户报告 joystick_control 持续等待 /controller_manager/init_finish。日志显示 18:59:08 启动的 sdk 中 ros_robot_controller 正常运行，但 servo_controller 在启动后立即以退出码 1 结束。只导入 controller_manager 复现 ModuleNotFoundError: No module named 'control_msgs'，错误来自 FollowJointTrajectory 导入。该依赖已经声明在 package.xml，故不重复修改声明或删除轨迹接口来掩盖缺包。

APT policy 显示 ros-jazzy-control-msgs 未安装且有候选版本；模拟安装计划仅新增此包，不升级或删除其他包。用户要求补齐后已尝试 sudo -n apt-get install -y ros-jazzy-control-msgs，但系统要求管理员密码，安装未发生。已在电脑 USB 手柄启动指南写明用户执行的安装命令、安装后只导入验证及保留现有底层驱动时单独补启动 servo_controller 的步骤。未向用户索取密码、未更改系统权限、未启动运动节点。系统安装与安装后导入验证仍待用户执行。

文档备份：`.migration-backup/2026-10-27-control-msgs-dependency-191036`。


## pygame 手柄轮询 API 修复（2026-10-27）

用户日志显示控制器已就绪、USB WirelessGamepad 已连接，随后因 `Joystick` 不存在 `get_attached()` 而退出。本机 pygame 2.5.2 不提供该方法；AVX2 性能警告不是本次异常原因。修改 `src/peripherals/peripherals/joystick_control.py` 的连接和输入回调：使用 `get_init()` 判断初始化状态，结合当前实例的 `JOYDEVICEREMOVED` 事件及 pygame 读取异常处理断连。没有调整按键映射、点动参数和原初始化/START 动作。

更新 `tests/test_pc_gamepad.py`，用 `spec_set` 限制模拟设备的 API，增加缺失 API、其他设备移除、输入错误回归覆盖。加载 `scripts/setup_pc.bash` 后运行手柄、速度、串口超时、移植和路径共五个测试文件，结果 **149 passed**。未加载环境时曾因自定义消息模块不可见而收集失败；加载后通过。`colcon build --base-paths src/peripherals --packages-select peripherals --symlink-install` 成功。真实 pygame 连接及轮询检查通过，舵机发布被 mock，未发送运动指令；不代表真实按键或机械臂运动已验证。

已更新 [电脑USB手柄控制排障与启动.md](电脑USB手柄控制排障与启动.md) 和手柄速度说明。用户可保留正常运行的底层控制链，仅结束旧手柄 launch 后重新启动；启动仍会发送 1 秒初始化姿态，需要先清空运动范围。此前 control_msgs 缺包记录是历史诊断；本次用户日志已通过控制器就绪检查，但没有另行核验系统包安装记录。

本次修改前备份：`.migration-backup/2026-10-27-pygame-api-191538/`，包含代码、测试和相关文档。恢复代码后需重新构建并重启，且会恢复该 API 崩溃，不建议作为运行方案。仍待用户完成实机短按点动验证。


## 遥控输入与控制输出可视化窗口（2026-10-27）

新增独立 `peripherals/gamepad_monitor` 窗口：只读读取电脑 Linux 手柄，订阅上层舵机目标、底层舵机指令和上层命令缓存，显示按钮/轴、各 ID 最近值、话题发布者数量、接收计数和事件时间线。默认自动保存 JSONL 与 CSV，支持停止/新建记录、暂停列表、清空列表和设备断连重试。明确区分初始快照、输入变化、控制目标与实际反馈；不按时间相近就宣称因果关系，不把缓存当实测位置。

新增 gamepad_monitor.py、gamepad_monitor_data.py、tests/test_gamepad_monitor.py 和 [遥控输入与控制输出监视器.md](遥控输入与控制输出监视器.md)。setup.py 注册入口，package.xml 声明窗口依赖。本机 Tk 8.6 可用，无系统安装操作；原 joystick_control 和控制协议没有修改。

新增 12 项测试与原测试共 **161 passed**，peripherals 定向构建成功。测试覆盖输入解析、断连重连、日志与错误处理、订阅消息转换和隐藏 Tk 窗口；未发送运动命令，未验证实际机械臂动作。另以隐藏窗口进行约 1.5 秒真实环境只读验证，成功读取 `/dev/input/js0`，收到 73 条命令缓存、0 条上层目标、0 条底层指令；未发送运动命令。备份 `.migration-backup/2026-10-27-gamepad-monitor-192341/`。启动命令、记录格式、边界及回退步骤见新说明。


## 窗口调节关节点动参数（2026-10-27）

用户要求在可视化窗口控制 joint_step 与 joint_duration。已开放这两项运行时参数，保持原类型和范围：整数步长 1–20 pulse、运动时间 0.02–30 秒。joystick_control 在参数校验阶段不修改内部值，只在完整事务成功后更新缓存；参数服务与输入回调同属默认互斥组。其他笛卡尔参数仍为只读；未改变按键映射、初始化和 START 动作。

窗口增加读取当前值和应用两项参数按钮，采用异步 GetParameters / SetParametersAtomically 服务。只有用户点击才发送请求，显示服务端确认值、拒绝原因或超时未确认状态，不将本地输入框当作已生效值。参数请求与结果纳入时间线和记录文件。目标可通过 --control-node 指定，默认 /joystick_control。更新仅保留在运行进程内，不写回启动配置；旧节点需重启一次加载新代码。

修改 joystick_control.py、gamepad_monitor.py、原测试和说明，新增 jog_settings.py、tests/test_live_jog_settings.py。新增 21 项测试，全部相关测试 **182 passed**；专用无硬件节点真实参数服务读写往返通过，peripherals 定向构建成功。没有向实际手柄节点提交参数或发送真实运动命令，实际点动效果待用户验证。

备份 `.migration-backup/2026-10-27-live-jog-settings/`。操作、首次升级重启要求、初始化动作提醒及回退步骤见 [遥控输入与控制输出监视器.md](遥控输入与控制输出监视器.md) 与 [手柄速度调节说明.md](手柄速度调节说明.md)。


## Python FK/IK 离线原型（2026-10-27）

用户要求基于已有机械参数编写 FK 与 IK 供查看。新增 `kinematics/python_kinematics.py`：依据 transform.py 和教程 Modified DH 表实现矩阵 FK、解析几何 IK、pitch 网格搜索、五关节 pulse 映射；不导入 ROS 或串口。工具沿最终 +Z 的偏移、基座坐标与几何 pitch 约定明确列为待厂商/实物验证，未复制教程有疑点的最终旋转矩阵，未混用不一致的 URDF 参数。保留原 .so 和现有服务节点导入，未自动启用新模型。

新增 `scripts/preview_kinematics.py`、`tests/test_python_kinematics.py`、[Python_FK_IK离线实现与验证.md](Python_FK_IK离线实现与验证.md)，生成 docs/assets 下两套 JSON/PNG 示意。默认输入及指定目标均可离线计算并画图，不发运动命令。

新增 23 项测试通过（含 400 组随机位置/完整旋转往返），全部相关测试 205 passed；kinematics 定向构建成功。验证只是算法和模型内部一致性，不证明原厂兼容性、实际定位精度或无碰撞。原型不是原 get_fk/get_ik 接口的直接替代；当前手柄控制与 enable_kinematics 设置保持原样。下一步先核对厂商输出或实物已知姿态，再决定服务后端接入。

既有文档备份：`.migration-backup/2026-10-27-python-fkik-prototype/`；新增文件不被原控制流程调用，不使用原型时不再运行预览脚本即可。
