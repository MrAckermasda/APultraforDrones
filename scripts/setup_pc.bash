#!/usr/bin/env bash
# Usage: source /path/to/ros2_ws/scripts/setup_pc.bash
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    echo '请使用 source scripts/setup_pc.bash，使配置作用于当前终端。' >&2
    exit 1
fi
export ARMPI_WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
_arm_ros_distro="${ARMPI_ROS_DISTRO:-${ROS_DISTRO:-jazzy}}"
if [[ ! -f "/opt/ros/${_arm_ros_distro}/setup.bash" ]]; then
    echo "未找到 /opt/ros/${_arm_ros_distro}/setup.bash；请设置 ARMPI_ROS_DISTRO。" >&2
    unset _arm_ros_distro
    return 1
fi
source "/opt/ros/${_arm_ros_distro}/setup.bash"
if [[ -f "${ARMPI_WS}/config/pc.env" ]]; then
    source "${ARMPI_WS}/config/pc.env"
elif [[ -f "${ARMPI_WS}/config/pc.env.example" ]]; then
    echo '未找到 config/pc.env；使用示例默认值。建议复制 config/pc.env.example 并配置串口。' >&2
    source "${ARMPI_WS}/config/pc.env.example"
fi
if [[ -f "${ARMPI_WS}/install/local_setup.bash" ]]; then
    source "${ARMPI_WS}/install/local_setup.bash"
else
    echo '尚未构建当前工作区，请先 colcon build，再重新 source 此脚本。' >&2
fi
unset _arm_ros_distro
