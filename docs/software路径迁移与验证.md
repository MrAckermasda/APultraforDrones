# software 路径迁移与验证

日期：2026-10-27（Asia/Shanghai）。对应请求：阅读高级控制教程、提供本机建议、调整 software 路径、定位遥控源码。

教程结论和遥控源码入口见 [FK_IK资料核对与遥控适配建议.md](FK_IK资料核对与遥控适配建议.md)。本次没有启动 GUI、机械臂节点或训练任务，没有改动 FK/IK 和手柄动作逻辑。

## 已实施的路径修改

1. **Python 资源路径**：原厂工具内 `/home/ubuntu/ros2_ws`、`/home/ubuntu/software`、`/home/ubuntu/third_party_ros2` 等资源路径改为根据 `__file__` 定位当前工作区。覆盖颜色配置、标定文件、手眼标定保存位置、训练/转换脚本等。仍保留旧调用依赖的尾部斜杠。
2. **GUI 启动脚本**：`arm.sh`、`lab_tool.sh`、`collect_picture.sh`、`servo_tool.sh`、`hand2cam.sh`、`calibration/tool.sh` 改为 Bash，从脚本位置定位 `scripts/setup_pc.bash`。统一加载 ROS 和当前工作区，正确传递含空格的路径；不再依赖 `.zshrc`。保留标定 UI/后端的原启动安排。
3. **GUI 内 ROS 命令**：`armpi_ultra_control/servo_controller.py` 使用 `subprocess.run` 参数列表调用 `ros2`，继承启动环境，不再另开 zsh 并读取旧 shell 配置。接收切换的原行为未重设计，仍需后续梳理串口所有权。
4. **独立 SDK**：`Servo_upper_computer` 和手眼标定目录中的两份本地 SDK 也支持 `ARMPI_SERIAL_PORT`、`ARMPI_BAUDRATE`，显式构造参数优先；默认仍为 `/dev/rrc` / 1000000。没有将代码替换成另一份协议实现。
5. **动作组配置**：`config/pc.env` 设置 `ARMPI_ACTION_GROUP_PATH` 默认指向当前工作区的 `software/armpi_ultra_control/ActionGroups`，已有环境变量优先。
6. **旧动作转换输入**：两个 `action_change.py` 默认保留逻辑上的旧 `software/arm_pc/ActionGroups` 输入目录，可用 `ARMPI_LEGACY_ACTION_GROUP_PATH` 覆盖。没有把旧格式转换器输入自动指向现成 ArmPi 动作组，以免重复交换舵机编号；该旧目录未随包提供，不应直接运行转换脚本。
7. **相机标定脚本**：启动脚本加载当前环境；保存脚本读取指定归档中的 `ost.yaml`，修改 `camera_name` 后写入当前工作区的 `camera_usb_info.yaml`，首次覆盖前备份原文件。不再依赖 CWD 或递归删除旧目录。此次只用合成归档在临时目录验证，没有改动真实相机标定值。
8. **桌面快捷方式**：4 个 `.desktop` 的 Exec/Icon 更新为实际文件位置；同时清理验证器指出的旧字段。未安装到用户桌面或应用目录，也未设置桌面信任属性。
9. **数据集入口**：两份活动 `data.yaml` 的根目录更新为当前数据路径；采图目录 ImageSets 列表改用相对于列表文件的 `./../JPEGImages/...`，保留条目顺序与训练/验证划分。未修改图片、标签或模型文件。

第三方 `ultralytics`、`roLabelImg` 源码没有批量替换。`software/data/runs/**/args.yaml` 是历史训练记录，原路径保留用于追溯；已有缓存文件没有作为运行路径配置直接修改。

## 再搬到另一台设备时

Python 路径和 Bash 启动脚本会随目录位置自动定位。桌面启动器和训练 YAML 需要明确的本机路径，新增脚本可刷新它们：

```bash
cd /新路径/ros2_ws
python3 scripts/refresh_software_paths.py
source scripts/setup_pc.bash
```

刷新脚本依赖 PyYAML，只修改列出的快捷方式、活动数据集配置和 ImageSets 文本列表，不启动程序，不安装桌面入口，不接触硬件。对当前路径重复运行应输出 `Refreshed 0 files`。

环境脚本保留已导出的 `ARMPI_ACTION_GROUP_PATH`；如果旧终端已设置旧路径，请重新 `export ARMPI_ACTION_GROUP_PATH="$ARMPI_WS/software/armpi_ultra_control/ActionGroups"`，或先 unset 再 source。

GUI 启动器本身已适配路径，例如 `bash software/armpi_ultra_control/arm.sh`，但主 GUI 仍会导入不兼容的 ARM FK/IK。路径正确并不表示界面全部功能可运行；本次不建议用启动 GUI 的方式检查路径。

## 验证

- 非第三方工具的 63 个 Python 文件通过 AST 语法检查。
- software 下相关 Bash 脚本通过 `bash -n`。
- 4 个桌面入口通过 `desktop-file-validate`。
- 原 11 项 PC 移植测试与新增 13 项 software 测试共 24 项通过。
- 新增测试覆盖：迁移后的资源根定位、带空格目录中的启动脚本参数传递、已知配置文件存在、桌面刷新幂等性、数据集顺序/标签不变、两份独立 SDK 环境变量配置（mock 串口）、相机标定保存和旧值备份（临时目录合成数据）。
- 未运行需要控制板、相机、手柄或模型的业务程序。系统仍缺少 pygame，检查时未检测到 `/dev/input/js*`；FK/IK 二进制架构问题未解决。

复查命令：

```bash
source scripts/setup_pc.bash
python3 -m pytest -q tests/test_pc_portability.py tests/test_software_paths.py
python3 scripts/refresh_software_paths.py
```

本轮未修改 ROS 包安装规则或节点代码，因此没有为路径修改重复进行全工作区构建；前次 17 包构建结果不等于本轮新增工具的实机验证。

## 备份与文件清单

40 个既有文件的修改前版本保存在 `.migration-backup/2026-10-27-software-paths/`，保留原目录结构，文件列表为该目录下的 `changed-paths.txt`。刷新脚本重复运行不会覆盖首份备份。

新增文件包括：

- `scripts/refresh_software_paths.py`
- `tests/test_software_paths.py`
- 本记录及 `docs/FK_IK资料核对与遥控适配建议.md`

需要回退时按文件清单恢复对应原文件，避免将上一次 ROS 包适配的备份与本轮 software 备份混用。
