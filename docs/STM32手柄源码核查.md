# STM32 手柄控制源码核查

日期：2026-10-27。目标：确认当前板载手柄遥控的调速位置。检查为离线文件读取，没有连接串口、启动控制节点、执行动作或烧录固件。

## 结论

找到 STM32 工程，但这份资料缺少可编辑的手柄控制模块。可以定位到 `ps2_set_action_time` 等目标文件符号，不能据此确定函数参数、当前默认值或生成可用的调速固件。此次只补充文档，没有修改运动代码和二进制。

用户确认手柄 USB 接收器插在控制板上。所附原理图的 USB-Host 电路使用 CH559T；USB 数据线接入 CH559T，其 `559_TX` 信号通向核心板接口。结合手柄目标文件引用 `huart3`，这些证据支持“手柄 → USB 接收器 → CH559T → 核心板处理”的设计。但原理图是否与实物版本完全一致、当前板内固件是否来自本工程，尚未核实。

## 资料与一致性

- 原始压缩包：`/home/lyl/3.源码资料/STM32/ArmPiUltra.rar`。
- 当前解压目录：`/home/lyl/ros2_ws/ArmPiUltra/`。
- 工程：`ArmPiUltra/MDK-ARM/ArmPiUltra.uvprojx`，声明目标 `STM32F103RB`、编译器 `ARMCLANG V6.16`。
- 本地原理图：`/home/lyl/2 STM32控制器资料/SCH_xArm Open Controller V2.0.pdf` 与 `SCH_STM32_Core Module.pdf`。前者 PDF 文件名为 V2.0，但图纸标题栏显示 ArmPiController V1.0，不能混用文件名与硬件版本。
- 使用系统 libarchive 只读遍历 RAR，共 1336 个条目；对非 Drivers 目录中的 `.c/.h/.uvprojx/.lib/.ioc` 共 78 个文件与工作区逐字节哈希核对，全部相同，无缺失或不同。因此下面的手柄源码缺失也存在于原压缩包，不能通过重新解压解决。
- Keil 项目列出 69 个文件路径，均可找到，但没有 ps2 手柄模块条目。这是“工程引用文件齐全”，不等于“当前遥控固件源码完整”。

## 手柄模块的证据

存在：

```text
ArmPiUltra/MDK-ARM/ArmPiUltra/ps2_porting.o
ArmPiUltra/MDK-ARM/ArmPiUltra/ps2_porting.d
```

`.d` 文件首部明确记录它当时依赖：

```text
Hiwonder/Portings/Src/ps2_porting.c
Hiwonder/Portings/Inc/ps2_porting.h
```

这两个源文件在原始压缩包和当前解压目录均不存在；本次也没有在 `/home/lyl/3.源码资料` 中找到同名文件。

用 `nm` 查看 `.o`，可见：

| 符号 | 能说明什么 |
|---|---|
| `ps2_init`、`ps2_handler` | 曾编译过手柄初始化和处理逻辑 |
| `ps2_joint_control` | 存在关节控制函数名 |
| `ps2_inverse_kinematics_control` | 存在笛卡尔/逆运动学控制函数名 |
| `ps2_set_action_time`、`ps2_get_action_time` | 存在动作时间设置/读取接口名，是下一步调速重点 |
| 未定义引用 `huart3`、`robot_arm_knot_run`、`robot_arm_coordinate_set` | 目标文件依赖串口及机械臂运动接口 |

`.o` 是已编译的 ARM 目标文件，不是 C 源码。符号表不能确认 `ps2_set_action_time` 的参数类型、单位、有效范围、默认值、按键映射或是否由串口协议对外开放。不能猜一个调用或直接修改二进制常量。

当前工程列表、`main.c` 和随包 `ArmPiUltra.axf` 符号表中没有接入该 ps2 模块的证据。`.o/.d` 应视作其他构建留下的线索，不能因其存在就认为这次工程会链接和运行手柄控制。还不能证明随包 `.hex` 与实物当前固件相同。

## 当前可读源码能确认的运动时间

1. `Hiwonder/Arm/src/robot_arm.c` 中 `robot_arm_knot_run(id, target_duty, time)` 将 `time` 传给底层，函数注释明确单位为毫秒。
2. `Hiwonder/Peripherals/Src/serial_servo.c` 中 `serial_servo_set_position(..., duration)` 将位置和 duration 的低/高字节编码到舵机指令；它是多个上层动作共用的底层函数。
3. `Hiwonder/System/packet_handle.c` 中 `packet_serial_servo_handle()` 的 `case 0x01` 直接使用上位机包中的 `cmd->duration`。
4. `Core/Src/main.c` 启动时调用 `robot_arm_reset(1500)`，这对应启动复位时间，不能当作手柄点动速度。
5. `packet_handle_init()` 当前注册 LED、BUS_SERVO、BUZZER、SYS 回调；`PACKET_FUNC_GAMEPAD` 虽出现在枚举里，但枚举声明不能证明有手柄实现或可用的遥控调速命令。

**不要通过给通用 `serial_servo_set_position()` 的 duration 统一乘倍数来修复手柄速度。** 这会改变上位机运动、复位或其他调用者的语义，而且在当前工程未接入手柄模块的情况下，仍无法建立实际遥控速度与改动之间的联系。

## 下一步修改所需文件

需要与当前控制板固件版本匹配的完整工程，重点补齐：

```text
Hiwonder/Portings/Src/ps2_porting.c
Hiwonder/Portings/Inc/ps2_porting.h
实际使用的 main.c、工程文件，以及这两个模块的依赖
```

拿到后先核对 `ps2_set_action_time` 的实现与调用点、关节点动步长、处理周期、释放/断连行为；保留现有默认值并将确实控制遥控速度的常量集中配置。若希望电脑运行时调速，还需确认是否有现成协议接口；当前没有找到，不能直接用 ROS 参数或任意串口命令代替。

完整构建后还需匹配板型和当前固件功能，再安排备份、烧录与实机验证。当前机器未检测到 `armclang` 或 `arm-none-eabi-gcc`，本次未进行固件构建；不存在“修改/烧录已生效”的结果。

电脑端 `peripherals/joystick_control.py` 的新增参数仍只适用于电脑端控制链，不能用来调整当前板载遥控。不要叠加启动电脑端手柄程序来验证固件速度。

## 与 FK/IK 的关系

新资料包含 `Hiwonder/Arm/Inc/kinematics.h` 和 `Hiwonder/Arm/src/LeArm.lib`，库符号有 `fkine`、`ikine`、`kinematics_init`。这是 ARM 编译库及接口，不是可在 x86_64/Python 3.12 直接加载的扩展，也没有因此补齐 FK/IK 算法 C 源码。不能把它改名替换 ROS 中的 `.so`；参数和模型是否与电脑端相符也需要另行核对。

## 已执行验证与回退

完成压缩包/解压目录一致性、项目引用完整性、源码调用检查、目标文件与 AXF 符号检查、原理图文本及图像核对。没有修改固件源码，不需要源码回退。此次更新的既有文档有备份，备份目录记录在 PC 迁移修改记录中。
