# FK/IK 实机小步验证

日期：2026-10-28。用户已确认实物满足当前几何条件，本轮据此推进执行工具；没有把该确认当作本轮独立测得的精度结果。代码验证环境是 Ubuntu 24.04 / x86_64 / ROS 2 Jazzy / Python 3.12.3。树莓派应从源码构建并复测。

## 新增能力

`kinematics_jog` 从控制板读取五个舵机的实际脉宽，使用指定 YAML 中的 FK/IK 参数规划相对位移；默认仅预览。显式指定 `--execute` 后，分段发送命令，每段等待、回读、验证，再继续下一段。它保留腕部转角及几何 pitch，不修改夹爪 ID 1，不写硬件偏移或卸力。

这是实机调试用的分段动作工具，不是连续实时轨迹控制器。当前位置和误差均由**实测关节脉宽 + 几何模型**计算；末端真实 XYZ 仍需要外部测量，不以 FK 结果代替真实 TCP 精度。

## 启动前提

- 实物几何、方向、零位以及工具中心点已核对；有标定 YAML 时在每条命令中指定同一 `--config`。
- 底座固定、空载、运动方向及整个路径没有碰撞；接近奇异位形或限位时停止试验。
- 仅保留一套底层控制板驱动。停止手柄、动作组、`servo_controller`/controller manager 等其他控制程序。
- 此工具不依赖 `/kinematics_python` 求解服务，测试时可关闭该服务，减少总线读请求。
- 物理断电/急停可用，注意断电可能导致机械臂下落。软件停止不是硬件急停。

## 构建

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
colcon build --packages-up-to kinematics --symlink-install
source scripts/setup_pc.bash
ros2 run kinematics kinematics_jog --help
```

本机已构建依赖时，也可仅构建 `--packages-select kinematics`。

## 终端 A：底层驱动

确认 `config/pc.env` 的串口路径和波特率正确，再运行；已有驱动时不重复启动。

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
ros2 launch ros_robot_controller ros_robot_controller.launch.py
```

驱动启动会打开串口并发送 PWM 偏差/底盘电机停止等初始化命令，不是纯只读检查。它不在构造函数中发送整臂总线舵机初始姿态。不要为这个试验启动整套 bringup 或 sdk，以免引入其他控制者和初始化动作。

## 终端 B：先预览相对当前位置向上 3 mm

```bash
cd /home/lyl/ros2_ws
source scripts/setup_pc.bash
ros2 run kinematics kinematics_jog \
  --delta-mm 0 0 3 --duration 2 \
  --report log/kinematics_jog/preview.json
```

若有独立标定配置，在命令末尾添加 `--config /绝对路径/my_kinematics.yaml`；默认使用安装包中的 `python_kinematics.yaml`。若底层驱动使用其他命名空间，添加 `--controller /实际节点名`，读取与命令话题都会使用这个前缀。

这条命令读取真实位置、规划并输出起点脉宽、目标 XYZ、终点脉宽、轨迹和 JSON 报告。**预览不会创建运动命令发布者**。向上是否有空间仍由现场确认。`--delta-mm` 使用基座坐标系，X/Y/Z 的方向沿用已核对的模型。

查看报告：

```bash
python3 -m json.tool log/kinematics_jog/preview.json
```

关键字段：`status: planned`、`executed: false`、`start_pulses`、`target_position_m`、`path`。ID 顺序始终为 `[6,5,4,3,2]`。

## 明确执行同一类小步动作

确认预览位移方向和路径可行后：

```bash
ros2 run kinematics kinematics_jog \
  --delta-mm 0 0 3 --duration 2 --execute \
  --report log/kinematics_jog/up3mm.json
```

**这条命令会驱动机械臂。** 每次运行都会重新测量、重新规划，并在发送前再次检查起点是否变化；它不执行之前 JSON 中的旧轨迹。预览后若机械臂已明显改变姿态，应重新预览。`--execute` 是此独立工具的执行开关，不依赖 YAML 中控制服务输出的 `enable_pose_targets`。

`duration=2` 是各段运动时间总和，不包含读数、等待稳定和检查耗时，实际通常更长。工具每段命令最长约 0.2 s，之后默认等待 0.1 s 并回读；下一段开始前再次回读。若误差超过条件，立即中止后续段，不自动重试动作，不自动回到起点。

## 限制与拒绝条件

| 检查 | 当前限制/行为 |
|---|---|
| 相对位移 | 非零，欧氏长度不超过 5 mm |
| 运动时间 | 1～10 s（不含测量/停顿） |
| 单关节总偏移 | 相对初始脉宽不超过 20 |
| 相邻规划点 | 每关节不超过 5 pulse |
| 实测到下一目标 | 不超过 10 pulse，且满足配置脉宽速度上限 |
| 整数化 | 沿用模型限位、FK 位置/pitch 误差、轨迹差分速度/加速度检查 |
| 起点变化 | 规划与执行前实测五关节不一致时拒绝 |
| 分段跟踪 | 默认误差最大 5 pulse，超过则中止 |
| 测量 | 五个 ID 完整、无重复、数据合法、从请求开始计时不过期；默认超时 1 s |
| 最终验收 | 关节回读 FK 相对目标的误差满足配置的 position/pitch tolerance |
| 并发 | 同一用户本机运行锁；每段检查直接命令话题是否有其他发布者，控制订阅者是否恰好一个 |

总线读取失败请先排查供电、设备、固件、ID 和通信。确有慢读而非失联时，可通过 `--read-timeout` 设置 0.1～5 s；调大也会增加检测故障前的等待时间，不应以此掩盖失联。

ROS 图检查不能提供跨机器的绝对排他控制，也不能阻止其他客户端在检查后突然加入。外部串口程序、固件内手柄或未经过同一 ROS 话题的控制源不在检测范围内。应实际停止其他控制源。

## 中断、故障和报告

执行中 Ctrl+C 或 SIGTERM 会尝试发送 ID 6、5、4、3、2 的 `stop` 命令。读数超时、位置偏差、命令冲突或报告写失败等异常，也会在可能已经运动时请求停止。

停止采用当前控制板 `SetBusServoState` 协议，`present_id=[1,id]`、`stop=[1]`。不会发送夹爪停止或卸力。命令发布成功不等于硬件停止确认；控制板/串口失联或进程被 SIGKILL 时不能保证停止。底层驱动也没有因此获得硬件看门狗。

每次预览、每个实测检查点和结束状态写入 JSON。`--report` 路径会被覆盖，建议每次执行使用不同文件名；省略时自动在 `log/kinematics_jog/` 使用时间戳和进程号命名。报告包括模型几何、参考脉宽、关节限位和配置路径。

- `completed`：所有命令段、关节回读和模型残差检查完成。
- `aborted`：中途失败；查看 `error`、最后一个 `checkpoints`、`stop_requested` 和 `stop_publication`。
- `stop_publication` 的 sent 仅表示发布过停止请求，不是实机停止回执。

**必须区分“关节跟踪通过”和“TCP 实物精度通过”。** 执行前后用相同基座坐标系测量实际 TCP，核对变化方向和距离。先验证一个方向的小步，再逐步验证其他方向；每次重新预览。不要直接重复执行命令累计位移来做长距离试验。

## 本轮验证与回退

自动化测试仅使用内存传输模拟和独立 ROS domain 的模拟控制板。已检查：默认预览不发布命令、每段回读、反馈 ID 乱序、超时、命令发布者冲突、起点变化、执行错误、报告写入失败、SIGINT/SIGTERM 中断后的停止协议，以及 JSON 结果。未连接实际串口，未执行机械臂运动。

本轮执行 `colcon build --base-paths src/driver/kinematics --packages-select kinematics --symlink-install` 成功；`ROS_DOMAIN_ID=175 python3 -m pytest -q tests` 为 **255 passed**（3 条已有 pygame/pkg_resources 警告）。安装后的 `ros2 run kinematics kinematics_jog --help` 正常，语法检查及 `git diff --check` 通过。

新文件：`kinematics/supervised_jog.py`、`kinematics/jog_cli.py`、`tests/test_supervised_jog.py`、`tests/test_jog_ros.py`；入口与 PyYAML 依赖加入现有包。回退只需不运行 `kinematics_jog`；没有修改默认启动入口、控制板固件或 EEPROM 标定。
