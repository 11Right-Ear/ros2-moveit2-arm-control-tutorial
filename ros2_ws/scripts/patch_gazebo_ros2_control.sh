#!/usr/bin/env bash
# 打补丁：修复 gazebo_ros2_control (Humble) 在加载 controller_manager 前
# 用 `--param robot_description:=<整段URDF>` 传参导致 rcl 参数解析失败的问题，
# 并把读回来的 NaN 关节速度兜底成 0。
#
# 背景一：gazebo_ros2_control 的插件源码里有这么几行
#   std::string rb_arg = std::string("robot_description:=") + urdf_string;
#   arguments.push_back(RCL_PARAM_FLAG);
#   arguments.push_back(rb_arg);
# URDF 是多行 XML，rcl 的 YAML 解析器无法把它当作一个标量解析，插件会打印
#   "Couldn't parse parameter override rule"
# 然后直接 return —— controller_manager 虽被创建，但服务不响应，
# spawner 全部失败，机器人无法控制。
#
# 背景二：gazebo_system.cpp 的 read() 里
#   this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);
# 用 position 控制方式时关节是被 SetPosition 直接摆过去的，ODE 不会去积分
# 关节的角速度，GetVelocity() 拿到的就是没初始化过的 NaN，
# 于是 /joint_states 里的 velocity 全是 NaN，RViz / MoveIt 那边会被 NaN 污染。
# 这里加一个 isnan 判断，把 NaN 兜底成 0。
#
# 新版 gazebo_ros2_control（master / Iron+）已改用参数服务器传递 robot_description，
# 第一个补丁不再需要。本脚本用于 Humble。
#
# 用法：
#   cd ros2_ws/src
#   git clone -b humble https://github.com/ros-controls/gazebo_ros2_control.git
#   ./patch_gazebo_ros2_control.sh
#   cd ..
#   colcon build --packages-select gazebo_ros2_control

set -euo pipefail

TARGET="${1:-gazebo_ros2_control/src/gazebo_ros2_control_plugin.cpp}"
SYSTEM="${2:-gazebo_ros2_control/src/gazebo_system.cpp}"

if [ ! -f "$TARGET" ]; then
  echo "错误：找不到 $TARGET" >&2
  echo "请先在 ros2_ws/src 下 clone gazebo_ros2_control (humble 分支)，再运行本脚本。" >&2
  exit 1
fi

if grep -q 'robot_description:=' "$TARGET"; then
  echo "找到待删除的 robot_description 传参代码，正在打补丁..."
  # 删除这 4 行（注释 2 行 + 代码 2 行），并保留其余内容
  sed -i '/set the robot description parameter/,/arguments.push_back(rb_arg);/d' "$TARGET"
elif grep -q 'rb_arg' "$TARGET"; then
  echo "找到 rb_arg 变量，正在打补丁..."
  sed -i '/std::string rb_arg = std::string("robot_description:=")/d; /arguments.push_back(rb_arg);/d' "$TARGET"
else
  echo "无需打补丁：$TARGET 里已经没有 robot_description 传参代码了。"
fi

if grep -q 'rb_arg' "$TARGET"; then
  echo "错误：补丁没有完全生效，请手动检查 $TARGET" >&2
  exit 1
fi

# ---- 关节速度 NaN 兜底 ----
if [ -f "$SYSTEM" ]; then
  if grep -q 'GAZEBO_NAN_VELOCITY_GUARD' "$SYSTEM"; then
    echo "无需打补丁：$SYSTEM 里已经有 NaN 速度兜底了。"
  else
    echo "给 $SYSTEM 打 NaN 关节速度补丁..."
    python3 - "$SYSTEM" <<'PY'
import sys

path = sys.argv[1]
s = open(path).read()

old = """      this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);"""
new = """      this->dataPtr->joint_velocity_[j] = this->dataPtr->sim_joints_[j]->GetVelocity(0);
      // GAZEBO_NAN_VELOCITY_GUARD: position 控制方式下关节是被 SetPosition
      // 直接摆过去的，ODE 不会积分关节角速度，GetVelocity() 会返回没初始化
      // 过的 NaN，/joint_states 的 velocity 就全是 NaN，这里兜底成 0。
      if (std::isnan(this->dataPtr->joint_velocity_[j]))
      {
        this->dataPtr->joint_velocity_[j] = 0.0;
      }"""

assert old in s, 'joint velocity anchor not found'
s = s.replace(old, new)
open(path, 'w').write(s)
print('NaN velocity guard applied to', path)
PY
  fi
fi

echo "补丁完成：$TARGET"
echo "现在可以执行： colcon build --packages-select gazebo_ros2_control"
