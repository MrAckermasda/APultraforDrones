# ArmPi Ultra ROS 2 工作区（PC 移植快照）

此仓库保存一份 ArmPi Ultra ROS 2 源码及本工作区的 PC 适配。已知本机验证平台为 Ubuntu 24.04 / x86_64 / Python 3.12.3 / ROS 2 Jazzy；这不代表其他平台或每个可选功能均已验证。控制板协议、固件版本及机械臂具体标定需以实际硬件复核。

## 快速准备

```bash
cd /path/to/ros2_ws
cp config/pc.env.example config/pc.env
# 编辑 config/pc.env，设置实际串口路径、底盘和摄像头
source scripts/setup_pc.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source scripts/setup_pc.bash
```

串口访问权限需由本机系统配置；不要把宽泛的 `chmod 777` 作为常规修复。请参阅 `docs/机械臂首次运动操作指南.md` 和 `docs/电脑USB手柄控制排障与启动.md`。**启动控制节点会连接硬件，有些入口会发送初始化姿态或运动命令**；先确认机械臂周围安全、只运行一套底层串口驱动，并按指南逐步启动。

## 主要内容

- `src/`：ROS 2 驱动、舵机控制器、手柄控制、监视器、运动学等源码。
- `software/`：上游应用与工具源码、动作组和资源；训练输出/模型权重按 `.gitignore` 排除。
- `ArmPiUltra/`：STM32 工程源码；Keil 等本机生成的编译输出不纳入版本。
- `scripts/`、`config/`、`docs/`、`tests/`：PC 配置、检查工具、迁移/操作记录和回归测试。

## 当前 PC 控制路径

手柄已由 PC 上的 pygame 读取，不需要把手柄 USB 接回 STM32。常用控制入口和安全前置条件见操作指南。遥控输入/输出窗口：

```bash
source scripts/setup_pc.bash
ros2 run peripherals gamepad_monitor
```

关节 jog 的 `joint_step` 与 `joint_duration` 可运行时调整；GUI 和日志中需区分用户输入、目标值、发送命令及上层命令缓存。`servo_states` 不等同于持续实测舵机反馈。

## Python FK/IK 与可选 ROS 后端

纯 Python FK/IK 已增加可独立部署的 ROS 2 服务后端，无需加载 ARM 专用 `.so`。它读取总线舵机实测位置，检查反馈时效、整数脉宽和轨迹完整性；默认节点为 `/kinematics_python`，IK 输出关闭，不发布运动命令。构建后可运行：

```bash
source scripts/setup_pc.bash
ros2 launch kinematics python_kinematics.launch.py
```

本机已通过模拟总线 ROS 服务联调，但实机几何、零位、工具偏置与方向尚待测量校准。部署参数、实测比对工具与启用步骤见 [Python FK/IK 部署与标定](docs/Python_FK_IK部署与标定.md)；离线模型推导与示例见 [离线实现与验证](docs/Python_FK_IK离线实现与验证.md)。

实物几何核对后，可用新工具做真实关节反馈驱动的小步验证，默认仅预览：

```bash
ros2 run kinematics kinematics_jog --delta-mm 0 0 3 --duration 2
```

需要已运行的底层控制板服务；加 `--execute` 才发送动作。分段运动、回读、故障停止和现场操作步骤见 [FK/IK 实机小步验证](docs/FK_IK实机小步验证.md)。本轮仅完成模拟总线验证，未实际驱动机械臂。

## 可选模型/大文件

为了避免普通 GitHub Git 推送包含超大文件，本快照不跟踪模型权重、推理引擎和训练输出；个别模型超过 GitHub 普通文件限制。原始架构 `.so`、STM32 编译产物和本机 colcon 输出同样排除。源码引用的可选模型/平台专用资源可能需从厂商或原部署另行取得，且其来源、许可及目标平台兼容性需独立核实。详见 `docs/版本快照与外部资源.md`。

## 检查

在迁移基线环境曾通过手柄、速度参数、PC 兼容性、串口模拟、路径和 Python FK/IK 相关测试（205 passed，记录见 `docs/PC迁移修改记录.md`）。该历史结果不是本次新运行，也不代表真实硬件运动学已校准。不同目标设备请重新构建并运行相应测试。
