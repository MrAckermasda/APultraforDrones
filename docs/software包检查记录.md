# software 包检查记录

日期：2026-10-27（Asia/Shanghai）

## 检查范围与结论

检查位置为工作区根目录 `software/`。此次读取代码、解析 Python 语法，并以 SQLite 只读模式检查动作组；没有启动 GUI、导入会打开串口的业务模块或发送机械臂指令。未修改 software 的程序或启动配置。

这个目录包含桌面控制、动作编辑、标定、颜色调参、采图、数据标注和训练工具。它补充了 ROS 工作区之外的原厂应用与数据，但没有找到正／逆运动学实现源码或兼容当前 x86_64 主机的替代二进制。主控制界面仍导入 `src/driver/kinematics` 所提供的运动学模块。

## 目录与依赖

| 目录 | 从源码确认的作用 | 与运动学的关系 |
|---|---|---|
| `armpi_ultra_control` | PyQt5 主控制界面，舵机滑块、位置/偏差读取、动作组编辑和回放、末端坐标显示与控制 | `main.py` 顶层直接导入 FK/IK，窗口构造创建 FK 对象；无兼容库时会阻塞整个界面启动 |
| `armpi_ultra_control/ActionGroups` | 已保存的舵机动作序列 | 24 个 `.d6a` SQLite 文件；动作数据本身不依赖 FK/IK，但现有主界面的初始化依赖它们 |
| `servo_tool` | 独立舵机调试界面 | 未发现直接 FK/IK 依赖，使用工作区 `ros_robot_controller` SDK；启动脚本、环境变量和接收控制仍需适配 |
| `Servo_upper_computer` | 独立舵机偏差调节界面 | 未发现直接 FK/IK 依赖，使用目录内自带的旧 SDK |
| `calibration` | 标定和位置微调界面及后端 | GUI 直接导入 FK/IK，也通过 ROS 调用 FK/IK 与标定服务 |
| `hand2cam_tf_matrix_software` | 相机与机械臂之间的标定工具 | 调用并等待 `/kinematics/set_joint_value_target` 和 `/kinematics/set_pose_target` 服务 |
| `lab_tool` | 颜色 LAB 阈值调节、图像显示 | 未发现直接 FK/IK 依赖；需要图像话题与 YAML 路径 |
| `collect_picture` | 订阅图像并采集图片 | 未发现直接 FK/IK 依赖；需要图像话题和相机配置 |
| `roLabelImg` | 图像标注工具源码 | 本次未逐文件审查第三方实现 |
| `ultralytics`、`yolov8`、`yolov11`、`data` | 检测/训练代码、模型及数据资源 | 不提供这台机械臂的 FK/IK 实现；本次未验证模型运行或训练依赖 |

`software/ultralytics/ultralytics/__init__.py` 标注版本为 `8.2.76`，这是附带源码版本，不等于本机已安装版本。

## 主控制界面的实际控制链

```text
armpi_ultra_control/main.py
  ├─ servo_controller.py → ROS 工作区中的 Board 类 → 直接打开串口
  ├─ forward_kinematics / inverse_kinematics → 末端显示与坐标控制
  └─ ActionGroups/*.d6a → 已保存的各舵机目标值和时间
```

关键证据：

- `main.py:16` 导入本地 `servo_controller`；该模块在第 4 行执行 `board = Board()`。导入 GUI 模块本身就可能打开串口，不能把“导入一下试试”当作无硬件副作用的检查。
- `main.py:24`、`:25` 直接导入两个运动学扩展，`:32` 创建 `ForwardKinematics()`。
- `fk_process()` 根据当前界面内的舵机位置值计算并显示末端坐标；`valuechange1()` 在发送舵机指令后调用它。因此仅删除两个 import 仍不足以让界面正常工作。
- `button_run()` 的 `ik_run` 分支读取 x/y/z、俯仰角等并调用 `get_ik()`，选出舵机位置后下发。
- `servo_controller.py` 通过 ROS 话题切换底层控制节点的接收状态，并控制 GUI 自己持有的串口接收线程。关闭接收不等于关闭串口；后续适配需要梳理 GUI 与 ROS 节点对同一串口的使用方式。

此前“直接舵机控制与动作回放本身不需要 FK/IK”的结论仍成立，但不表示这个原厂 GUI 能在缺少库时原样启动。恢复基本界面可以考虑让运动学成为可选能力，禁用末端坐标显示/操作并保护所有 FK 调用；完整坐标控制仍需兼容的求解库。

## 动作数据已找到

`software/armpi_ultra_control/ActionGroups` 中有 24 个 `.d6a` 文件，全部可只读打开并查询 `ActionGroup` 表。样例 `OK.d6a` 有 6 条动作记录，列为：

```text
Index, Time, Servo1, Servo2, Servo3, Servo4, Servo5, Servo6
```

这验证了文件与表的可读性，不表示动作已在当前机械臂上验证。动作数据不是 FK/IK 算法源码。

前一轮迁移时仅检查 ROS 源码树，动作组被列为待补资源；现在已经在 software 中找到。后续使用 ROS 应用的动作回放器时，可以将 `ARMPI_ACTION_GROUP_PATH` 指向 `$ARMPI_WS/software/armpi_ultra_control/ActionGroups`。原主 GUI 使用自身目录下的动作组，不读取这个环境变量。

## 迁移缺项

1. **主界面仍受 ARM FK/IK 阻塞。** software 内未找到运动学源码、同名替代 Python 实现或 `.so` 文件。
2. **启动路径尚未适配。** `arm.sh` 当前指向 `/home/lyl/software/armpi_ultra_control/main.py`，实际目录为 `/home/lyl/ros2_ws/software/armpi_ultra_control/main.py`；`arm.desktop` 还指向旧的 `/home/ubuntu/software/armpi_fpv_control/`。其他 `.sh`、`.desktop` 也有旧用户名和目录。
3. **shell 依赖。** 部分脚本依赖 zsh、`~/.zshrc`；主界面的接收切换也在 zsh 中执行 ROS 命令，需要改为明确加载当前工作区或通过程序接口调用。
4. **独立 SDK 尚未改。** `Servo_upper_computer/ros_robot_controller_sdk.py` 默认 `/dev/rrc`、1000000，不具备前一轮给 ROS SDK 添加的环境变量覆盖功能。`pc.env` 的串口配置不会自动作用于这份旧代码。
5. **资源路径仍旧。** `lab_tool`、`calibration`、手眼标定等保留 `/home/ubuntu/...` 路径；部分代码直接读取环境变量，未设置时会失败。
6. **运动学接口范围扩大。** 主界面及标定界面还导入 `get_position_ik`，虽然此次搜索未发现实际调用，替换求解器或整理无用导入时仍需核对；不能仅实现 ROS 节点当前导入的函数而忽略这些 GUI 的导入要求。

## 已完成的验证

- 排除第三方 `ultralytics`、`roLabelImg` 后，63 个 Python 文件通过 AST 语法解析。
- 24/24 个动作数据库可只读查询。
- 当前 Python 可以定位 `PyQt5`、`cv2`、`rclpy`、`yaml`、`numpy`；这不是 GUI 启动或图形环境兼容性的完整验证。
- 未启动桌面应用，未验证摄像头、串口协议、舵机动作或标定效果。

后续可优先适配主控制界面的基本舵机与动作组功能，并将 FK/IK 相关功能明确设为不可用；待取得兼容运动学库后再恢复坐标控制和标定。

## 后续路径适配（2026-10-27）

上述“路径尚未迁移”等内容是首次检查的历史状态。后续已调整原厂工具路径、启动环境、独立 SDK 串口配置和快捷方式，详见 [software路径迁移与验证.md](software路径迁移与验证.md)。FK/IK 架构问题和 GUI 对运动学的硬依赖仍未解决；没有进行实机运行验证。
