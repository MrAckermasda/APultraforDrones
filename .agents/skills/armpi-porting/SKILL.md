---
name: armpi-porting
description: 迁移和适配本仓库的 ArmPi Ultra ROS 2 机械臂包到电脑、树莓派或其他 Linux 设备；检查串口、ROS/Python/CPU 兼容性、资源路径及外设驱动，记录迁移结果。
---

# ArmPi Ultra 移植

适用于包含 `src/driver/ros_robot_controller`、`servo_controller`、`kinematics`、`sdk` 的这套机械臂工作区。以目标设备实测为准，区分已经验证的功能与仅完成构建的功能。

## 先核对环境与连接

- 读取目标系统 `/etc/os-release`、`uname -m`、Python 版本、`ROS_DISTRO`、工作区配置和已有迁移记录；2026-10-27 的迁移基线是 Ubuntu 24.04 / x86_64 / ROS 2 Jazzy / Python 3.12.3，不是所有目标设备必须固定的版本。
- 优先运行仓库 `scripts/check_portability.py --core-only`；它只读设备元数据和 ELF 文件头，不打开串口。完整扫描可检查语音等可选库。匹配 CPU 不等于匹配 Python ABI。
- 使用 `/dev/serial/by-id`、`/dev/serial/by-path`、`udevadm info` 记录设备。CH340 的 VID/PID 只能识别串口转换器，不能证明所连设备使用这套机械臂协议。多块没有唯一序列号的同型号设备优先按物理端口或额外属性区分。
- 树莓派原始发行版、控制板固件、相机型号未提供时记录为未知，不从包内统一的 `0.0.0` 版本号推断。

## 本仓库接口约定

- 控制板链路是 ROS 节点 → `Board` → USB 串口 → 控制板，不应仅因原主机是树莓派而引入 GPIO 库。
- `Board` 的协议默认波特率为 `1000000`，默认设备 `/dev/rrc`。覆盖入口是 ROS 参数/launch 参数 `serial_port`、`baudrate`，以及 `ARMPI_SERIAL_PORT`、`ARMPI_BAUDRATE`。显式参数优先于环境变量；检查包装类确实传递参数。
- `ros_robot_controller` 使用 `ros_robot_controller_msgs`，上层 `/servo_controller` 使用 `servo_controller_msgs`；两种舵机位置消息不能互换。保持消息字段、串口协议、舵机 ID、方向、脉宽范围和标定参数；更换电脑本身不构成修改机械参数的依据。
- `CHASSIS_TYPE` 影响底盘启动和运动学底座高度；`Slide_Rails` 与普通底座不同。默认 `None` 表示独立机械臂，按实物调整为 `Mecanum`、`Tank` 或 `Slide_Rails`。
- `CAMERA_TYPE=usb_cam` 是普通 RGB 相机默认路径，`aurora` 使用原深度相机分支；普通 RGB 图像不能替代深度数据。USB 设备可用 `ARMPI_VIDEO_DEVICE` 或 `video_device` 覆盖；检查分辨率、像素格式、内参、TF 和实际话题。

## 路径、配置与依赖

- 在新工作区重新构建；不要沿用其他 CPU / Python / ROS 环境的 `build`、`install`、`log`。已有目录先移开备份。
- launch 资源使用 `get_package_share_directory()`；旧 `need_compile` 开关已移除。不要替换成另一个用户的绝对目录。
- 当前资源布局保留源码：`source scripts/setup_pc.bash` 自动设置 `ARMPI_WS` 并读取 `config/pc.env`。运行时标定 YAML 等经 `sdk.paths.workspace_path()` 定位，部分程序会写回源码配置。纯 install 部署需要另行设计可写配置目录和资源安装规则。
- 本机参数集中在 `config/pc.env`，已设置的环境变量优先。迁移时重核串口和硬件型号；`ARMPI_ROS_DISTRO` 可以覆盖 ROS 路径，但不能使不兼容的软件自动兼容。
- 动作组目录通过 `ARMPI_ACTION_GROUP_PATH` 指定；动作文件、相机厂商驱动、模型、语音库可能未包含在此包中。外部资源缺失要报告真实缺项。
- `package.xml` 应声明实际运行依赖，重点检查 `pyserial`、NumPy、OpenCV、PyYAML、`transforms3d` 和自定义消息。尽量使用目标 ROS 对应的系统 Python；不要用覆盖系统 NumPy 的方式盲目修复二进制依赖。
- 两份 `99-ttyACM0.rules` 应一致。仓库规则匹配原控制板 `1a86:55d4`、权限 `0660`、组 `dialout`；本次 PC 检测的是 `1a86:7523`，不匹配该规则。不要把所有 `ttyACM*` / `ttyUSB*` 都映射成 `rrc`，也不要沿用 `0777`。

## 二进制迁移的硬限制

原包 `forward_kinematics.so`、`inverse_kinematics.so` 及语音扩展为 ARM aarch64，不能直接用于 x86_64。本仓库未发现正／逆运动学扩展的构建源码；需要厂商提供目标架构、Python ABI 匹配的库，或取得源码后重新编译。不能通过改名、复制到 install 或更改扩展名解决。不要为让启动通过而返回伪造的运动学结果。

`sdk armpi_ultra.launch.py enable_kinematics:=false` 可隔离运动学节点，但依赖 IK 的抓取、分拣、视觉定位仍不可用。TensorRT engine、模型插件、语音库和相机驱动也需要分别核对目标平台。

## 验证与交付

1. 先做语法检查、XML/launch 参数解析、消息与核心包构建，以及 `tests/test_pc_portability.py` 中的模拟串口测试。
2. 检查 ELF 架构、必要模块导入、串口权限和话题类型。构建成功不能替代这些验证。
3. 在准备好的硬件上再验证通信，随后验证单关节和上层动作。普通节点构造可能打开串口并发命令；尤其 `bringup/startup_check.py` 会发送初始化姿态，不是无副作用的检测程序。未准备实机时使用 mock，不通过启动整套 bringup 来“只检查一下”。
4. 在 Markdown 中记录日期、环境、改动路径、配置优先级、实际执行的检查、未解决项、下一步和回退方式。串口权限或管理员操作未完成时明确区分“脚本已提供”和“系统已生效”。
