# Gazebo 真实物理仿真问题排查与解决

> 目标：让本项目的 `my_robot`（6 自由度机械臂 + 夹爪）从 **mock 假硬件**（`mock_components/GenericSystem`）
> 迁移到 **Gazebo 真实物理世界**（`gazebo_ros2_control/GazeboSystem`），
> 在 Gazebo 里**看得见、控制得动**，并且夹爪能通过终端指令开合。

本文记录整个迁移过程中踩过的坑、每个坑的根因、以及**为什么某些“看起来对”的方法行不通、最终的方法为什么行得通**。

---

## 一、问题总览

| # | 现象 | 根因 | 最终方案 |
|---|------|------|----------|
| 1 | Gazebo 里看不见机械臂（只剩底座） | URDF→SDF 转换用了 Gazebo Classic 不认的 `relative_to` | 自己写 `urdf_to_sdf.py`，把位姿换成 1.6 语义 |
| 2 | 模型 spawn 后炸开 / 塌掉 / 关节变 NaN | ODE 默认步长 0.001 让接触约束求解器发散 | 步长减半到 0.0005 + 调接触参数 |
| 3 | 用 PID 力矩控制时关节速度立刻 NaN | ROS 线程和物理线程抢 ODE 关节，无线程互斥 | 去掉 PID，改 position 控制 |
| 4 | 偶发模型被“掰飞” | 残留的 robot_state_publisher 节点让插件读到过期 URDF | 专用节点名 + 启动前清理残留进程 |
| 5 | `/joint_states` 速度全是 NaN | 没声明 velocity 状态接口 | 补 velocity 状态接口 + 脚本兜底 |
| 6 | 夹爪不能开合 | 之前把夹爪改成 fixed 回避了 prismatic NaN | 修好根因后恢复 prismatic + mimic |
| 7 | `gripper_controller` inactive | 容器里 `my_robot_description` 没重新 build | 重新 colcon build |

下面逐个展开，重点讲 **“失败的方法为什么失败”** 和 **“成功的方法为什么成功”**。

---

## 二、问题 1：Gazebo 里看不见机械臂（只剩底座）

### 现象
启动后 Gazebo 里只能看到 `base_link` 和一根圆柱，其余 7 个连杆（shoulder/arm/elbow/forearm/wrist/hand/夹爪）全部消失。

### 根因
ROS 2 里 spawn 的是 URDF，但 Gazebo 实际加载的是 **SDF**。中间要经过一次 URDF→SDF 转换。
sdformat 自带的转换器输出的是 **SDF 1.7 写法**，用 `relative_to="..."` 来表达连杆/关节的相对位姿：

```xml
<link name='shoulder_link'>
  <pose relative_to='joint1'>0 0 0 0 0 0</pose>
</link>
```

但 **Gazebo Classic 11 的物理引擎只实现了 SDF 1.6 的位姿语义**，解析不了指向 link/joint 的 `relative_to`。
于是所有连杆的位姿全部塌回模型原点 (0,0,0)，表现为“只剩底座”。

### 为什么直接“降版本”行不通
第一反应是用 `gz sdf -v 1.6 -p` 强制输出 SDF 1.6。但实测**没用**：

- sdformat 的“降版本”只是**把 `relative_to` 属性丢掉**，并没有把位姿重算；
- link 的位姿本来写的是“相对 joint 的偏移”，丢掉 relative_to 之后，这个偏移被当成“相对模型原点”的偏移；
- 结果 link 位姿变成 `0 0 0`（也就是所有连杆仍叠在原点），比原来还糟。

**教训**：降 SDF 版本号 ≠ 降位姿语义。位姿的“参照系”必须真正重算，不能只删属性。

### 成功的方法
自己写一个转换器 `my_robot_description/scripts/urdf_to_sdf.py`，做**真正的运动学正解**：

1. 解析 URDF 的关节结构（父子关系、origin、axis）和 `<ros2_control>` 里的 `initial_value`（出生姿态）；
2. 从根连杆开始做正解，算出每个连杆在**模型坐标系下的绝对位姿**；
3. 关节的 `<pose>` 写成**关节坐标系相对子连杆**的位姿（这是 Gazebo Classic 里 `joint <pose>` 的真实含义：`WorldPose = anchorPose + 子连杆位姿`）；
4. 只调用 `gz sdf -p` 拿几何/质量/惯量，位姿部分全部重写。

这样生成的 SDF 是 Gazebo Classic 真正能读懂的写法，机械臂就立起来了。

---

## 三、问题 2：ODE 物理发散，模型炸开 / 塌掉

### 现象
位姿修好之后，模型能 spawn，但没过多久 link 就飞到几百米外，关节状态变成 NaN，整条机械臂塌成一团。

### 根因
Gazebo Classic 默认 `max_step_size = 0.001`（每秒 1000 步）。在这个步长下，ODE 的接触约束求解器对**退化接触**（比如底座底面正好贴在 z=0 的地面上、面面共面）会发散，接触力算爆，模型被甩飞。

### 失败的方法
早期试过给链接加质量、调 PID、加阻尼、改碰撞盒留缝——这些都是**在“症状”上打转**，没有解决接触约束发散这个本质问题，所以模型还是时不时炸。

### 成功的方法
在 `my_robot_bringup/worlds/empty_ros.world` 里把步长减半并放宽接触约束参数：

```xml
<physics name="default" type="ode">
  <max_step_size>0.0005</max_step_size>
  <real_time_update_rate>2000</real_time_update_rate>
  <real_time_factor>1</real_time_factor>
  <ode><constraints>
    <contact_surface_layer>0.001</contact_surface_layer>
    <contact_max_correcting_vel>100</contact_max_correcting_vel>
    <contact_max_correcting_pos>0.1</contact_max_correcting_pos>
  </constraints></ode>
</physics>
```

同时 spawn 时把底座抬高 1cm（`-z 0.01`），让模型“落下来”而不是“贴上去”，避免底座和地面共面的退化接触。

---

## 四、问题 3：PID 力矩控制线程竞态 → 速度 NaN → 炸

### 现象
配置里一旦用了 `pos_kp` / `pos_kd`（PID 力矩控制），关节速度会立刻变成 NaN，随后模型炸开。

### 根因
`gazebo_ros2_control` 的 PID 分支会从 **ROS 线程**直接调用 ODE 的 `dJointAddHingeTorque()` 给关节加力矩，
而 Gazebo 的**物理线程**同时也在积分同一个关节，两边**没有拿物理引擎的 `physicsUpdateMutex` 做互斥**。
于是 ODE 的关节内部状态被并发写坏，关节速度立刻 NaN，模型被判定发散后炸开。

### 为什么“加互斥锁”不是首选
理论上给 `read()`/`write()` 加物理互斥锁也能解决，但这要改 `gazebo_ros2_control` 源码并重编，
代价高、且每次升级都要重打补丁。

### 成功的方法
**绕开力矩控制**，改走 position 控制分支：

- 把 xacro 里的 `pos_kp` / `pos_kd` 全部去掉；
- `gazebo_ros2_control` 检测到没有 PID 参数时走 **`SetPosition(0, cmd, true)`** 分支，直接设置关节角；
- `SetPosition` 不参与 ODE 的力矩求解，天然不会和物理线程抢状态，模型稳定。

这是本次最关键的转折：**之前夹爪 prismatic 关节的 NaN，根子就在这里**（见问题 6）。

---

## 五、问题 4：残留的 robot_state_publisher 导致过期 URDF

### 现象
偶尔启动后模型被用错误的初始角度“掰飞”，表现为关节速度 NaN。

### 根因
`gazebo_ros2_control` 是按**节点名**去连 `robot_state_publisher` 拿 `robot_description` 参数的。
如果之前 Gazebo 被强杀（关窗口、断电），会留下**孤儿的 robot_state_publisher 进程**。
机器上同时有多个同名节点时，插件可能连到旧的那个，读到**过期的 URDF**，
于是用错误的 `initial_value` 去掰关节，模型被物理引擎甩飞。

### 成功的方法
1. 给 `robot_state_publisher` 起一个**专用节点名**（`my_robot_state_publisher`），并在 URDF 的 `<robot_param_node>` 里保持一致；
2. 在 launch 文件的清理步骤里，启动前 `pkill` 掉残留的 `gzserver`/`gzclient` **以及** 同名残留 RSP 节点。

---

## 六、问题 5：`/joint_states` 速度全是 NaN

### 根因
关节只声明了 `position` 状态接口，没有 `velocity` 状态接口。
`joint_state_broadcaster` 拿不到速度，就往 `/joint_states` 里发 NaN 速度，RViz / MoveIt 会被 NaN 污染。

### 成功的方法
在每个关节的 `<ros2_control>` 块里补上：

```xml
<state_interface name="velocity"/>
```

另外在 `patch_gazebo_ros2_control.sh` 里加了一段兜底：`read()` 读回的关节速度如果是 NaN 就置 0。

---

## 七、问题 6：夹爪不能开合（本次核心工作）

### 背景：夹爪为什么之前被“砍掉”
夹爪原本的设计是（见 `gripper.xacro` 的历史版本）：

- 左手指：`prismatic` 关节，轴 +x，范围 0~0.06，**可控制**；
- 右手指：`prismatic` 关节，`mimic` 联动左手指（`multiplier="-1.0"` 反向）。

但在早期调试阶段，发现夹爪 prismatic 关节在 Gazebo 里返回 NaN，mimic 联动也不稳定，
于是当时做了两个“简化”提交：

1. `f40e51a`：去掉 mimic、去掉速度状态；
2. `14f9fba`：干脆把夹爪整个改成 `fixed`，并从 ros2_control / spawner 里移除。

### 为什么“改成 fixed”不是真正的解决
改成 fixed 只是**把夹爪控制整个砍掉**，属于“回避问题”而不是“解决问题”。
当时真正的根因其实是**问题 3（PID 力矩线程竞态）+ 问题 2（ODE 步长发散）**：
整个模型还处在不稳定的状态，prismatic 关节的 NaN 只是这个全局不稳定的**一个症状**，而不是 prismatic 关节本身的病。
但当时这两个根因还没被定位出来，所以误判成了“prismatic/mimic 有问题”。

### 成功的方法
在把手臂稳定性（步长 + 位置控制）真正修好之后，**原样恢复教程的夹爪设计**：

1. `gripper.xacro`：两个手指从 `fixed` 改回 `prismatic`
   - 左手指：`axis +x`、`limit 0~0.06`、`damping 1.0`，可控制；
   - 右手指：`mimic joint="gripper_left_finger_joint" multiplier="-1.0"`，由 Gazebo 物理引擎按 mimic 关系自动反向跟随。
2. `my_robot.gazebo.ros2_control.xacro`：把 `gripper_left_finger_joint` 加进 `GazeboSystem` 硬件接口
   （position 指令 + position/velocity 状态，`initial_value=0`）。
3. `ros2_controllers_gazebo.yaml`：新增 `gripper_controller`（`JointTrajectoryController`，控制 `gripper_left_finger_joint`）。
4. `gazebo.launch.py`：加一个 `gripper_controller` 的 spawner。

**为什么这次能成功**：因为 NaN 的真正根因（ODE 步长、PID 线程竞态）已经在前面修掉了，
prismatic + mimic 关节本身没有问题，恢复原设计即可，无需重新造轮子。

### 语义与测试指令

- `gripper_left_finger_joint = 0.0` → 张开；`= 0.06` → 闭合（右手指反向联动）。
- 演示闭合建议发 `0.05`（留 0.01 缝），避免两指贴死造成退化接触。

```bash
# 夹（闭合）
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_left_finger_joint], points: [{positions: [0.05], time_from_start: {sec: 2, nanosec: 0}}]}"

# 放（张开）
ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [gripper_left_finger_joint], points: [{positions: [0.0], time_from_start: {sec: 2, nanosec: 0}}]}"
```

---

## 八、问题 7：`gripper_controller` inactive（最后一个坑）

### 现象
代码改完、重新 launch 后，`ros2 control list_controllers` 显示：

```
gripper_controller  joint_trajectory_controller/JointTrajectoryController  inactive
```

并且报错：

```
Not acceptable command interfaces combination:
  Not existing: gripper_left_finger_joint/position
```

### 根因
**不是代码错，是容器里 `my_robot_description` 这个包没重新 build/install。**

- `gazebo_ros2_control` 插件只加载了 `joint1`~`joint6`，根本没加载 `gripper_left_finger_joint`；
- Gazebo 把两个手指当成 `fixed` 关节“焊”进了 hand_link（日志里的 `hand_link_fixed_joint_lump__gripper_...` 就是铁证）；
- 说明 `xacro` 处理时用的还是**旧的 fixed 版** `gripper.xacro`。

为什么控制器 yaml 是新的、URDF 却是旧的？因为 `ros2_controllers_gazebo.yaml` 在 `my_robot_bringup` 包里，
而两个 xacro（`gripper.xacro`、`my_robot.gazebo.ros2_control.xacro`）在 `my_robot_description` 包里，
后者没被重新安装。

### 成功的方法
重新 build 并验证安装目录里的 xacro 已经是新版：

```bash
cd /workspace/ros2-moveit2-arm-control-tutorial/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_description my_robot_bringup
source install/setup.bash

# 验证：两条都应该输出非 0
grep -c "gripper_left_finger_joint" install/my_robot_description/share/my_robot_description/urdf/my_robot.gazebo.ros2_control.xacro
grep -c "prismatic" install/my_robot_description/share/my_robot_description/urdf/gripper.xacro
```

---

## 九、最终验证结果

无头模式（`gui:=false`）实测通过：

- 所有 link 位姿正确：`base=0, shoulder=0.1, arm=0.6, elbow=1.2, forearm=1.3, wrist=1.8, hand=1.85, gripper_base=1.87`；
- 三个控制器全部 `active`：`arm_controller`、`gripper_controller`、`joint_state_broadcaster`；
- 发轨迹指令后机械臂关节跟踪到位（`joint2=0.6`、`joint3=0.4` 都在 `/joint_states` 里跟踪到）；
- 夹爪通过 `/gripper_controller/joint_trajectory` 能开合；
- `/joint_states` 的 velocity 不再出现 NaN。

---

## 十、经验教训

1. **降版本号 ≠ 降语义**：SDF 的 `relative_to` 问题不能靠 `-v 1.6` 解决，必须真正重算位姿的参照系。
2. **NaN 往往只是症状**：关节速度/位置 NaN 的根因可能是物理步长、线程竞态等更底层的东西，别急着在关节层面打补丁。
3. **“砍掉功能”不是修复**：把夹爪改成 fixed 只是回避问题；根因修好之后，原设计直接就能用。
4. **物理仿真优先调步长和接触参数**：Gazebo Classic + ODE 里，模型炸开先怀疑步长和退化接触，而不是去加质量、加阻尼。
5. **改完 xacro 记得重新 build 对应的包**：`my_robot_description` 和 `my_robot_bringup` 是独立包，改哪个就重装哪个，否则会出现“改了但没生效”的假象。
