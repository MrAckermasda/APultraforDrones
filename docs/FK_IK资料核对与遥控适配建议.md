# FK/IK 教程核对、本机修改建议与遥控源码

日期：2026-10-27（Asia/Shanghai）

## 资料与范围

阅读资料：`/home/lyl/1 教程资料/3.机械臂高级控制/3.机械臂高级控制.pdf`，共 31 页，以下页码为 PDF 文件页序。已提取正文，并对含图形公式的关键页面做图像核对。

教程提供几何模型、公式与调用示例；教程中的关闭自启动、启动 SDK、发布舵机命令等是资料内容，不是本轮需要执行的指令。本轮没有执行这些动作，也没有修改 FK/IK 求解器或手柄控制逻辑。本轮实际修改是 software 路径、启动环境及相关连接配置，详见另一份修改记录。

## 从教程确认的模型

PDF 第 16 页的 DH 表与 `src/driver/kinematics/kinematics/transform.py` 中的 Modified DH 注释一致：

| 关节 | α(i−1)，度 | a(i−1)，米 | d(i)，米 | θ 范围，度 |
|---|---:|---:|---:|---|
| 1 | 0 | 0 | 0 | −120 ～ 120 |
| 2 | −90 | 0 | 0 | −180 ～ 0 |
| 3 | 0 | 0.10048 | 0 | −120 ～ 120 |
| 4 | 0 | 0.100 | 0 | −200 ～ 20 |
| 5 | −90 | 0 | 0 | −120 ～ 120 |

当前 `transform.py` 另有如下参数，不能仅凭上述表格遗漏它们：

- 普通底座 `base_link=0.094605 m`；滑轨底座 `0.162 m`。
- `link1=0.10048 m`、`link2=0.100 m`。
- `link3=0.055 m`、`tool_link=0.115 m`；源码注释说明计算工具长度时使用二者之和 `0.170 m`，实际变换方向仍需与末端坐标系核对。
- 程序关节范围比教程名义范围各放宽 0.2°，注释解释为数值容差；不要再无依据扩大机械范围。
- `pulse2angle()` 返回弧度；服务 `SetRobotPose` 使用米为位置单位、度为 pitch/pitch_range 单位。`angle2pulse()` 接受关节弧度解的集合。
- 五个运动学关节对应舵机 ID `[6,5,4,3,2]`，夹爪开合是 ID 1。教程第 30 页示例和当前配置一致；“6 个舵机”不能直接当作 6 个独立末端位姿自由度。

## 教程提供了实现依据，但不能原样照抄所有公式

### 正解：第 17–18 页

可以按照第 18 页给出的逐关节变换矩阵连乘，再补齐基座和工具变换，构建不依赖 CPU 架构的 NumPy 实现。

注意该页随后引入了 `θ2+θ3+θ4=0`。这只能作为特定姿态的约束；关节轴平行不意味着转角和始终为零，不能用这个简化代替通用 FK。

该页最终印出的 T(0−5) 旋转部分也存在明显问题。按打印内容取 θ1=θ5=0，可得：

```text
R = [[1, 0,  0],
     [0, 1,  1],
     [0, 0, -1]]
```

本机数值复核：`det(R)=-1`，`R.T @ R != I`。因此该打印结果不是有效的刚体旋转矩阵，应视为需要复核的笔误/推导问题，而不是可直接执行的实现规范。建议使用原始逐节矩阵推导并测试，不凭印刷最终矩阵猜测修正项。

### 逆解：第 24–25 页

教程给出平面三连杆的几何法，包含腕点扣除、余弦定理、双分支和肩/腕角恢复：

```text
xw = x − l3*cos(α)
yw = y − l3*sin(α)
c = (xw² + yw² − l1² − l2²) / (2*l1*l2)
θ2 = ±acos(c)
θ1 = atan2(yw,xw) − atan2(l2*sin(θ2), l1+l2*cos(θ2))
θ3 = α − θ1 − θ2
```

这里的 x/y 是简化平面内坐标，θ1/θ2/θ3 是该平面模型的角度，不能直接对应到 ROS 世界坐标或完整 DH 关节编号。移植到完整机械臂时，需要处理底座旋转、基座高度、末端偏移、零位和方向转换、角度范围、不可达目标及多解连续性。

`|c|>1` 超过浮点容差时应报告无解；不能把真正不可达的点强行裁剪成可达。手柄连续移动还需要选择接近当前姿态的解，避免不同分支跳变。

### ROS 服务与示例：第 19–23、26–31 页

教程展示了服务和 demo，但仍导入 `ForwardKinematics`、`get_ik`，没有提供这两个扩展的完整实现源码。现有 ARM `.so` 的架构问题仍然存在。

以当前源码行为为准：普通 `set_pose_target` 返回求解结果，调用方随后发布舵机位置；`set_joint_value_target` 输入位置值并计算末端 Pose，本身没有下发舵机动作。教程中服务用途的简述不能替代对代码副作用的核对。

## 本机建议：按遥控目标分阶段处理

### 第一阶段：先完成单舵机手柄控制

无需先重写 FK/IK。建议对 `joystick_control.py` 增加明确的纯关节模式：

1. 将 FK/IK 服务客户端的创建/等待限定在启用末端模式时；在没有运动学时禁止 MODE 切换到末端坐标模式并给出明确提示。
2. 默认不在节点构造时发送初始化姿态，把摆位放在明确的按键/参数控制下。
3. pygame 事件处理采用有限的一次轮询，放在定时器内；删除当前定时器回调里的无限 `while True`，避免长期占用执行器线程。
4. 选定一种输入来源。现有控制节点直接读 pygame，不订阅 `/joy`；launch 同时启动 `joy_node` 并不是其必须条件。可保留 pygame 并简化 launch，或将控制逻辑统一改为订阅 `/joy`，不要混淆两条输入链。
5. 舵机状态按 ID 构建映射，不依赖消息数组的隐含顺序；检查状态有效性、连接丢失、按键释放和持续按压行为。
6. 区分“已发送的目标位置”与“实际位置反馈”。当前 `ServoManager.get_position()` 返回内部记录，`set_position()` 会更新发送值；它并不持续读取真实舵机位置。手动搬动或重新上电后不能无条件将记录当作实测姿态。
7. 在接入动作前验证按键布局、初始位置、脉宽边界和移动速率。当前下层虽会裁剪到 0–1000，也不等于所有机械姿态均不会相撞。

本轮仅提出这些修改点并定位源码，没有将其实现或声称遥控器已可用。

### 第二阶段：恢复末端坐标遥控

可向厂商索取 x86_64/Python 3.12 库；也可依据教程、`transform.py` 和实机参数编写纯 Python FK/IK。后者需要完成以下接口及验证：

- 保留 `ForwardKinematics.get_fk/set_link/get_link/set_joint_range/get_joint_range` 和 `get_ik/set_link/get_link/set_joint_range/get_joint_range` 的调用约定；GUI 还导入 `get_position_ik`，应核实是否实现或删除其未使用导入。
- 明确位置单位、角度单位、四元数结构、解集合结构和空解返回约定。
- 原 `.so` 与新 `.py` 使用相同模块名时，要明确隔离原二进制或改用显式后端模块，不能只把同名 `.py` 放旁边就假定一定加载它。
- 离线验证 FK→IK→FK 的位置与姿态一致性、旋转矩阵正交性、角度/脉宽往返、边界与不可达点、多解和奇异位置。
- 用原树莓派上的可用库导出同一组关节/目标点结果作对照（如仍有可用设备），再与实际已知姿态交叉检查。测试输入与输出采集本身无需发运动命令。

有了这份教程，重新实现运动学有了更具体的依据，但仍不能跳过坐标约定和实机验证就替换到运动链。

## 遥控源码定位

本仓库明确实现的主要遥控方式是 USB 游戏手柄。若实际使用的是航模/SBUS 遥控器，则需要另一条输入映射，不能把两者的入口混用。

| 源码 | 用途 |
|---|---|
| `src/peripherals/peripherals/joystick_control.py` | 手柄连接、pygame 事件、按键映射、单舵机/末端模式 |
| `src/peripherals/launch/joystick_control.launch.py` | 当前手柄 launch，同时启动 joy_node 和控制节点 |
| `src/peripherals/setup.py` | 注册 `joystick_control = peripherals.joystick_control:main` |
| `src/driver/servo_controller/servo_controller/actions.py` | START 调用的回位动作等 |
| `src/driver/servo_controller/servo_controller/bus_servo_control.py` | 发布 `/servo_controller` 舵机位置消息 |
| `src/driver/servo_controller/servo_controller/controller_manager.py` | 上层舵机目标处理和状态发布 |
| `src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py` | 底层串口控制；也发布 `~/sbus`、`~/joy` |
| `src/peripherals/peripherals/demo.py` | 仅打印 `/joy` 的示例，不驱动机械臂 |

当前手柄链路：

```text
USB 手柄/接收器 → pygame → joystick_control
    → /servo_controller → controller_manager → ros_robot_controller → 控制板

末端模式额外需要：当前位姿 FK → 坐标增量 → IK → 舵机目标
```

源码默认 mode=0；识别的手柄名称包括 `SHANWAN Android Gamepad` 和 `USB WirelessGamepad`。当前读取 `/dev/input/js0`、pygame `Joystick(0)`，还不是可配置的稳定设备选择。

按代码回调，未校验实物按键印刷与顺序：

| 按键 | 单舵机模式 | 末端模式 |
|---|---|---|
| 十字键上下 | ID 5 ±10 | x ±0.005 m |
| 十字键左右 | ID 6 ±10 | y ±0.005 m（左右符号以代码为准） |
| L1 / L2 | ID 1 −10 / +10，夹爪 | z +0.005 / −0.005 m |
| R1 / R2 | ID 3 +10 / −10 | pitch −1° / +1° |
| 方块 / 圆圈 | ID 2 +10 / −10 | 仍直接控制 ID 2 |
| 叉 / 三角 | ID 4 −10 / +10 | 仍直接控制 ID 4 |
| MODE | 切换模式 | 切换模式 |
| START | 回位 | 回位 |

“±10”是现有代码一次回调的位置增量，不表示已经实现稳定的按住连发速度。普通摇杆轴事件会更新数组，但当前 `axes_callback()` 实际处理的是十字键。

### 当前运行阻塞与源码问题

- 本机检查未找到 `/dev/input/js*`；尚未检测到可用手柄节点。
- 系统 Python 当前未安装 pygame。若要补齐此运行依赖，用户终端可执行 `sudo apt-get install python3-pygame`；本轮未安装。
- 原手柄构造函数无条件等待 FK/IK 服务，当前不兼容库会导致其无法完成初始化。
- 构造函数在初始化完成后会发布整臂初始姿态，启动不是纯读取操作。
- 原 launch 的 `disable_servo_control=True` 在控制源码中没有相应参数读取，不能把它当作有效的禁止运动开关。`max_linear/max_angular` 等也不是当前实现的速度配置入口。
- SDK 已有的 `enable_kinematics:=false` 只关闭运动学/抓取节点，不会自动改变手柄对这些服务的等待行为。
- 对 SBUS：已找到底层发布代码，未在 `app`、`example`、`peripherals` 中找到现成的 SBUS→机械臂控制映射。

当前入口命令是 `ros2 run peripherals joystick_control` 或 `ros2 launch peripherals joystick_control.launch.py`，这里只作源码入口标识，不作为本轮已经具备运行条件的操作建议。


## 官方公开仓库是否需要 FK/IK 扩展（2026-10-27 补充核对）

用户观察官网没有 forward_kinematics.so / inverse_kinematics.so，但演示似乎仍可运行。本次核对对象是 Hiwonder/ArmPi-Ultra 官方公开 GitHub 仓库，不等同于用户尚未提供链接的某个官网页面。

固定核对版本：`d43c2bce1681fa4f0828c4f33c981a959fa56f3b`。GitHub API 返回的递归目录树 `truncated=False`：kinematics 目录有 transform.py、kinematics_control.py、kinematics_demo.py、search_kinematics_solutions_node.py 等，但没有两个算法模块对应的 .so 或 .py 文件。

证据：

- 官方 `.gitignore` 含 `*.so`，公开仓库配置排除了这类二进制文件；该规则本身不能证明完整镜像的具体内容。
- 官方 `driver/kinematics/kinematics/search_kinematics_solutions_node.py` 第 16、17 行仍导入 `kinematics.forward_kinematics` 和 `kinematics.inverse_kinematics`，第 128、145、307 行调用 FK，第 172 行调用 IK。这些是实际算法依赖，没有自动切换到纯 Python 备用实现的分支。
- 官方 README 说明支持树莓派 5，程序已预装在机器人系统内，可直接运行。合理推断：公开源码仓库不等于完整可运行镜像；算法模块可能由预装环境或配套资料提供。本次没有检验官方完整镜像，不能声称已确定库的安装位置、Python ABI 或具体交付方式。
- 本机文件头仍确认两个 .so 为 ARM aarch64；无法直接供 x86_64 Python 使用。本次未导入这些库或启动运动节点。

本地使用边界：

| 功能 | 对这两个扩展的依赖 |
|---|---|
| 当前单关节手柄点动、调 joint_step/joint_duration | 不需要；直接生成舵机目标 |
| 按已知舵机位置发送固定姿态或动作序列 | 该路径不需要；若外层启动同时拉起运动学节点，应关闭该节点 |
| 本套 search_kinematics_solutions 节点 | 当前实现需要两种算法模块，顶层导入即发生依赖 |
| 从关节角计算末端位置、按 XYZ/俯仰求关节目标 | 需要 FK/IK 能力；现有实现由这两个扩展提供 |

transform.py 目前提供连杆/关节参数、角度与 pulse 映射以及旋转表示转换，不包含完整机械臂 FK/IK 求解器。kinematics_control.py 只是组装 SetRobotPose / SetJointValue 服务请求，不能替代算法模块。

更准确的结论是：基础关节控制不需要这两个 .so；保留当前运动学节点实现则需要可用的同名算法模块。数学上并不强制用 .so，也可在获得并验证完整几何、关节零位/方向/限制后实现兼容的 Python 或其他算法后端；目前仓库中没有这样的可替代求解实现。不能仅删掉 import 或改文件名就恢复坐标控制。

本次只追加核对记录，未修改运动代码，也未测试机械臂运动。

来源（固定版本，可复核）：

```text
https://api.github.com/repos/Hiwonder/ArmPi-Ultra/git/trees/d43c2bce1681fa4f0828c4f33c981a959fa56f3b?recursive=1
https://github.com/Hiwonder/ArmPi-Ultra/blob/d43c2bce1681fa4f0828c4f33c981a959fa56f3b/.gitignore
https://github.com/Hiwonder/ArmPi-Ultra/blob/d43c2bce1681fa4f0828c4f33c981a959fa56f3b/driver/kinematics/kinematics/search_kinematics_solutions_node.py
https://github.com/Hiwonder/ArmPi-Ultra/blob/d43c2bce1681fa4f0828c4f33c981a959fa56f3b/README.md
```

读取方式：GitHub API 目录树及 contents 接口。raw.githubusercontent.com 在本机返回证书链校验错误，改用正常通过证书校验的 GitHub API 完成内容核对，未禁用 TLS 验证。


## 后续进展：已完成离线 Python 原型（2026-10-27）

已根据上述模型实现独立 `kinematics.python_kinematics.ArmKinematics`，可计算 FK、几何 IK 和有界 pitch 搜索，附离线脚本、JSON/PNG 及测试。新模块不与旧 .so 同名，没有替换现有服务。23 项运动学测试通过（包含 400 组随机往返），全套相关测试 205 passed，kinematics 构建成功。

这改变的是“已有可供评审的 Python 算法实现”，不改变“缺少原厂源码、原 ARM 二进制不能在本机直接使用、尚无实机验证”的事实。工具方向和 pitch 定义明确属于待核验约定，不能把数值往返一致当作硬件校准完成。详见 [Python_FK_IK离线实现与验证.md](Python_FK_IK离线实现与验证.md)。
