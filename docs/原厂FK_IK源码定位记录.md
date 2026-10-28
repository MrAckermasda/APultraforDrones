# 原厂 FK/IK 完整源码定位记录

日期：2026-10-28。范围：本地工作区、可见 Git 历史、用户目录下原始产品资料、ROS2 ZIP 与 STM32 RAR，以及二进制调试信息。未联网检索，不据此断言上游没有公开源码。未加载或执行 ARM 二进制，也未打开串口。

## 结果

本地提供的原始发行资料没有找到 `forward_kinematics.py`、`inverse_kinematics.pyx` 或对应生成 C 源码。当前完整可读的 `python_kinematics.py` 是本项目后来重写的实现，不是还原的原厂源文件。

二进制保留的原厂构建目录为：

```text
/home/ubuntu/kinematics/src/hiwonder_kinematics
```

该路径来自两个 `.so` 的 DWARF `DW_AT_comp_dir`；本机不存在该目录。它指向编译时使用的目录，不能证明交付的树莓派镜像保留了该工程。

| 二进制 | DW_AT_name 编译单元 | 字符串中的原始源文件名 |
|---|---|---|
| forward_kinematics.so | forward_kinematics.c | forward_kinematics.py |
| inverse_kinematics.so | inverse_kinematics.c | inverse_kinematics.pyx |

两库均为 AArch64 ELF，保留 debug_info，含 `_cython_3_0_9` 标记和 `/usr/include/python3.10` 构建路径。

## 检查过的发行资料

1. `/home/lyl/3.源码资料/ROS2/Armpi_ultra-main.zip`：检查 2434 个 ZIP 条目，运动学目录仅包含 `.so`、调用封装、ROS 服务节点、参数/变换及包安装文件，没有核心 `.py/.pyx/.c`。包内两份 `.so` 的 SHA-256 与当前工作区逐一相同。
2. `/home/lyl/3.源码资料/Armpi_ultra-main/`：已解压的同一资料，搜索核心类/函数定义未找到源码。
3. `/home/lyl/3.源码资料/STM32/ArmPiUltra.rar`：读取 1336 个 RAR 条目，相关文件是 `Hiwonder/Arm/Inc/kinematics.h` 和 `Hiwonder/Arm/src/LeArm.lib`，没有对应算法实现源码。解压工程的 `MDK-ARM/ArmPiUltra.uvprojx` 引用该库；`robot_arm.c` 调用 `kinematics_init`。CMSIS 的矩阵求逆代码不是机械臂 IK 源码。
4. `/home/lyl/1 教程资料/3.机械臂高级控制/3.机械臂高级控制.pdf`：PDF 第 17～18 页给出 FK 分析/矩阵；第 24～25 页给出几何法 IK 分析；第 22、29 页的示例导入编译库。未找到库的完整函数实现。
5. 当前仓库全部可见 Git 历史：未找到原厂核心 `.py/.pyx/.c`。
6. 另外检查了本地下载、armpi_ultra、pi、hiwonder-toolbox、基础控制教程等相关目录中的候选文件；未找到目标源文件。

## 二进制指纹

| 文件 | SHA-256 |
|---|---|
| `forward_kinematics.so` | `97ad0b041ddf5dd01906d35f20138b9a38ae1dffef734ec44055a2e7be61d713` |
| `inverse_kinematics.so` | `49dc231be7cc2a021988347b5708fa4d113d583cf296ba9e2ceb9e816bd2f1ba` |

## 向厂家索取的准确内容

> 请提供 ArmPi Ultra ROS2 所附 `forward_kinematics.so`、`inverse_kinematics.so` 对应的完整 `hiwonder_kinematics` 构建工程。两库调试信息记录的目录为 `/home/ubuntu/kinematics/src/hiwonder_kinematics`，编译单元分别为 `forward_kinematics.c` 和 `inverse_kinematics.c`，源文件名分别出现 `forward_kinematics.py` 与 `inverse_kinematics.pyx`。请包含原始 `.py/.pyx/.pxd`（实际存在的文件）、构建配置、依赖及版本、模型参数/坐标约定和使用许可。如果只能提供 Cython 生成的 `.c`，请同时提供可复现的构建脚本及依赖说明。

生成的 C 文件若完整，可作为重编译输入，但不能把它当作可直接运行的简洁 Python 源码。重新构建后仍需数值回归和实机参数验证。

## 在原厂设备/镜像上进一步查找

如果还有原厂树莓派、容器或系统镜像，优先检查上述编译目录。可在对应设备上只读搜索：

```bash
find /home /opt /workspace -type f \
  \( -name 'forward_kinematics.py' -o -name 'inverse_kinematics.pyx' \
     -o -name 'forward_kinematics.c' -o -name 'inverse_kinematics.c' \) \
  2>/dev/null
```

本次未连接原厂设备，未检查未挂载的镜像内部，也没有联系厂家。
