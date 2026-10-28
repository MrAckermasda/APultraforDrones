# Python FK/IK ROS 2 部署与标定

日期：2026-10-27，Asia/Shanghai。验证环境：Ubuntu 24.04、x86_64、Python 3.12.3、ROS 2 Jazzy。

后续更新（2026-10-28）：新增分段执行及真实关节回读工具，见 [FK/IK 实机小步验证](FK_IK实机小步验证.md)。以下保留后端首次交付状态；本轮用户已确认实物几何条件，但仍未由本轮执行实机运动。

## 交付范围与状态

本轮增加可部署的纯 Python ROS 2 服务后端，运行于连接机械臂控制板的 Linux 主机（电脑或安装相应 ROS 2 的树莓派），不需要加载厂商 ARM `.so`。这不是向 STM32 刷入 Python 程序，也没有改写控制板固件。树莓派上的安装、性能和串口通信仍需在目标设备复测。

已完成 ROS 构建、真实 ROS 消息/服务联调和模拟舵机反馈验证。尚无独立实物尺寸、TCP 测量、方向/零位或原厂 FK 对照数据；未执行实机运动。软件可部署，模型还不能宣称已通过实机验收。

默认节点名为 `/kinematics_python`，`enable_pose_targets=false`。FK 和测量查询可用于标定；IK/轨迹输出及 `init_finish.success` 需要显式开启并收到完整有效的实测反馈。节点不创建运动发布者，不打开串口，不自动启动控制器。其读取请求通过已运行的 `/ros_robot_controller/bus_servo/get_state` 服务执行。

## 文件与算法变化

- `python_kinematics.py`：保留 Modified DH FK、解析 IK；增加五关节的标定参考脉宽 `zero_pulses`，名义默认值不变。
- `deployment.py`：按 ID 排序与验证反馈；期望 pitch 优先，然后按 pitch 距离搜索；取整后重新验证角度限位、位置误差和 pitch 误差；保留当前腕部转角 q5。
- `python_kinematics_node.py`：独立 ROS 后端，复用现有 `kinematics_msgs` 服务及控制板读取接口。
- `config/python_kinematics.yaml`、`launch/python_kinematics.launch.py`：安装到包 share 的部署资源。
- `scripts/check_kinematics_measurements.py`：独立实测 TCP 位置比对工具，不连接串口。

## 构建与启动

新设备使用源码重新构建，不复制其他 CPU/Python 的 `build` 或 `install`。按工作区 README 安装依赖后：

```bash
source scripts/setup_pc.bash
colcon build --packages-up-to kinematics --symlink-install
source scripts/setup_pc.bash
ros2 launch kinematics python_kinematics.launch.py
```

以上启动仅运行求解节点。没有底层控制板服务时 FK 仍可调用，实时位姿和 IK 则失败；不会阻塞节点启动。

在另一个已加载环境的终端检查 FK（不发送运动命令）：

```bash
ros2 service call /kinematics_python/set_joint_value_target \
  kinematics_msgs/srv/SetJointValue \
  '{joint_value: [500.0, 600.0, 825.0, 110.0, 500.0]}'
```

这些是计算输入，不代表要求机械臂运动到此姿态。

机械臂控制板驱动的通信准备和启动遵循 `机械臂首次运动操作指南.md`；本轮未启动驱动。使用一套经过确认的底层驱动即可，不同时运行多个串口访问者。若使用 `sdk armpi_ultra.launch.py`，保持 `enable_kinematics:=false` 关闭其中的原厂节点，再独立启动 Python 后端。

## 参数与标定

复制配置到工作区内自己的 YAML，用 `config:=/绝对路径/my_kinematics.yaml` 加载。文件使用 `/**: ros__parameters:`，改名节点仍能读取。启动参数 `enable_pose_targets` 覆盖 YAML，默认始终为 false；其他参数取 YAML，未给出的值取节点默认。参数为启动配置，在线 `ros2 param set` 会返回失败，防止只改参数显示值而求解器不更新。

| 参数 | 单位/含义 |
|---|---|
| `base_height` | m；未配置时 `CHASSIS_TYPE=Slide_Rails` 为 0.162，其他支持底座为 0.094605 |
| `upper_arm`, `forearm` | m；两个主连杆长度 |
| `wrist_length`, `gripper_length` | m；两者沿末端 +Z 合并，工具轴假设须实物验证 |
| `zero_pulses` | 顺序 ID `[6,5,4,3,2]`，分别对应参考角 `[0,-90,0,-90,0]°` 时的脉宽 |
| `joint_limits_deg` | 依次为 q1..q5 的 min/max，共十个数 |
| `feedback_service` | 测量服务，可改命名空间；禁止用指令位置缓存冒充实测反馈 |
| `feedback_timeout` | s；从总线请求开始计时，默认 1.0 |
| `feedback_period` | s；默认 0.2，同一时间最多等待一个请求 |
| `position_tolerance`, `pitch_tolerance` | m/度；量化脉宽回代模型的误差上限，默认 0.002/0.5，不是实机精度保证 |
| `max_velocity`, `max_acceleration` | pulse/s、pulse/s²；平滑轨迹实际整数序列的差分上限 |
| `trajectory_dt`, `max_samples` | 默认 0.02 s、501 点；超过工作量上限直接拒绝 |

脉宽映射保持原代码方向和 0.24°/pulse：`q_deg = (pulse-zero_pulses) * [0.24,-0.24,0.24,-0.24,0.24] + [0,-90,0,-90,0]`。这些参考偏移是主机模型校准，不写舵机 EEPROM，不改变硬件偏移。若硬件已经配置偏移，不要再次重复补偿。

至少采集三个不同且分布合理的姿态，记录**实测**五关节脉宽与 TCP 在机械臂基座坐标系的 XYZ（m），另行核对工具朝向与 pitch 正负号。示例数据格式如下，`null` 必须替换为真实测量值；工具会拒绝占位值：

```json
[
  {"pulses": [null, null, null, null, null], "position_m": [null, null, null]},
  {"pulses": [null, null, null, null, null], "position_m": [null, null, null]},
  {"pulses": [null, null, null, null, null], "position_m": [null, null, null]}
]
```

```bash
python3 scripts/check_kinematics_measurements.py measurements.json \
  --config src/driver/kinematics/config/python_kinematics.yaml --tolerance-m 0.005
```

滑轨使用 `--chassis Slide_Rails`，或在同一配置显式指定 `base_height`。检查器输出每个姿态和最大位置残差；超差退出 1，非法输入退出 2。默认 5 mm 仅是比对门槛，按测量精度和任务需求调整。它不自动拟合参数，不验证姿态、碰撞或动态性能，不会自动开启 IK。不要用 FK 合成的位置冒充实测数据。

## 服务契约和控制接入

服务名和消息类型与原节点相同，默认命名空间不同：

| 服务后缀 | 行为 |
|---|---|
| `set_joint_value_target` | 输入 ID 6..2 的五个脉宽；返回 FK Pose，单位 m、四元数 XYZW |
| `get_current_pose` | 从完整且未过期的总线测量计算 FK |
| `set_pose_target` | XYZ + 几何 pitch；返回五个整数脉宽，保留当前 q5 |
| `set_pose_target_smooth` | 五次时间插值的笛卡尔路径；返回展平的 N×5 脉宽数组 |
| `get_link` / `get_joint_range` | 查询本节点实际模型 |
| `set_link` / `set_joint_range` | 原子替换；非法参数不改变原模型；合法变更清除反馈并关闭 IK，需将新配置保存到 YAML 后重启开启 |
| `init_finish` | IK 已启用且有有效实测反馈时 success=true |

`pitch` 是工具方向相对 q1 径向水平面的几何俯仰，0 向外、+90 向上、-90 向下；不是 FK 欧拉角中的 pitch。`rpy` 是取整后 FK 姿态的固定轴 XYZ 欧拉角，单位度。任意六维姿态不在五轴 IK 的能力范围内。

IK 接受 `-180 <= pitch_range[0] <= pitch <= pitch_range[1] <= 180`。搜索最多 361 个规则网格点，外加首选 pitch 和上界；未找到表示当前采样网格无解，不证明连续范围无解。首选 pitch 下选择脉宽变化量最小的可表示解。

平滑轨迹含起点和终点：`N = max(1, ceil(duration/trajectory_dt)) + 1`，实际时间间隔 `duration/(N-1)`；必须每五个数分组，不能将整条数组当作单个目标。每个中间点都使用前一个解作为 seed，并检查相邻点脉宽速度和包含起停的加速度差分。不满足条件时返回 `success=false, pulse=[]`，不会跳过点或返回截断路径。整数化可能让短周期加速度较大，不能把脉宽差分限制当作完整动力学保证。单点服务不执行轨迹/速度检查，执行器需要另行规划。

原上层程序可能假定不同的 wrist、pitch/rpy、成功标志和轨迹点数；不能仅因消息类型相同就认定行为与厂商库完全等价。核对调用方后，才可用 `node_name:=kinematics` 提供原服务地址。不要与原节点并行占用相同服务名。

完成标定、控制板通信确认和调用方核对后，可以显式开启求解：

```bash
ros2 launch kinematics python_kinematics.launch.py \
  config:=/绝对路径/my_kinematics.yaml enable_pose_targets:=true
```

此命令仍不发送运动指令。获得的解只能说明数学与脉宽条件通过，执行前还需桌面/机身/夹爪碰撞检查、夹持负载评估和低速小范围验证。本实现没有碰撞模型、完整动力学约束、急停执行器或实机闭环轨迹执行器。

## 验证与回退

本轮执行：

```bash
source scripts/setup_pc.bash
colcon build --base-paths src/driver/kinematics --packages-select kinematics --symlink-install
ROS_DOMAIN_ID=173 python3 -m pytest -q tests
ros2 launch kinematics python_kinematics.launch.py --show-args
python3 -m compileall -q src/driver/kinematics/kinematics scripts/check_kinematics_measurements.py
git diff --check
```

结果：定向构建成功；已实际启动安装后的 launch，通过 FK 服务取得默认示例 XYZ `(0.1311233567, 0, 0.1016408255)` m，并验证重复 SIGINT 下干净退出。完整 `tests/` 为 **231 passed**；3 条现有 pygame/pkg_resources 警告。ROS 测试使用真实 rclpy 服务和独立 domain 的模拟控制板，无串口、无运动命令。覆盖解析 FK/IK、400 组随机回代、标定偏移、整数化限位/残差、反馈 ID 乱序/缺失/重复/失效、完整路径/零距离/无解中断/时长及速度约束、服务拒绝与恢复、默认 IK 禁用和测量比对工具。

`check_portability.py --core-only` 仍报告两份原厂 `.so` 是 ARM aarch64，与本机 x86_64 不符。新后端不导入这些库；原厂启动入口仍需匹配的库，不能据新后端构建成功宣称原路径已修复。

回退：停止 `python_kinematics` 节点即可；本轮没有更换默认运动学 launch、修改控制板串口协议、改舵机 ID 或启用任何动作。配置/在线标定服务不写 EEPROM。要继续使用厂商后端，仍须在匹配其 CPU/Python ABI 的环境使用原入口。
