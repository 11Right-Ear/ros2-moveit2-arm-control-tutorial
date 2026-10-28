import os
import subprocess
import sys
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    """在 Gazebo 真实物理世界里启动 my_robot 机械臂。

    启动顺序/职责：
      0) 清理残留的 gzserver/gzclient —— 上一次运行若没正常退出，新实例会因
         11345 端口被占用而起不来，此时屏幕上看到的还是旧 Gazebo 窗口，
         容易误判为"模型没更新"。
      1) gazebo          —— 启动 Gazebo 世界（默认空世界，带 GUI）
      2) robot_state_publisher —— 处理 my_robot.gazebo.urdf.xacro，发布 TF；
                            gazebo_ros2_control 插件也从它这里取 URDF 来建立
                            controller_manager
      3) spawn_entity    —— 把 URDF 转成 SDF 后 spawn 进 Gazebo；模型里的
                            gazebo_ros2_control 插件会创建 controller_manager
                            并加载控制器参数
      4) spawner x2      —— 激活 joint_state_broadcaster / arm_controller

    控制方式（仿真起来后）：
      ros2 topic pub /arm_controller/joint_trajectory \
        trajectory_msgs/msg/JointTrajectory "..." --once
    """
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = True

    # robot_state_publisher 专用节点名：gazebo_ros2_control 插件靠它去取
    # robot_description 参数（urdf 里的 <robot_param_node> 必须和这里一致）。
    rsp_node = 'my_robot_state_publisher'

    clean_gazebo_arg = DeclareLaunchArgument(
        'clean_gazebo',
        default_value='true',
        description='Kill leftover gzserver/gzclient processes before starting.'
    )
    # 只跑物理仿真、不要图形界面时用 gui:=false（例如在没有 X 的机器上跑 CI）
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start gzclient (Gazebo GUI).'
    )
    # 必须用 `pkill -x`（精确匹配进程名），不能用 `-f`（匹配整条命令行）。
    # `-f` 会同时匹配到两样东西：
    #   1) 本进程自己的命令行（"bash -c pkill -9 -f gzserver ..." 里就含 gzserver）
    #      -> bash 自杀，日志里 exit code -9；
    #   2) 刚启动的那个 gzserver（cmdline 是 "gzserver /opt/ros/humble/..."）
    #      -> 被误杀 -> /spawn_entity 服务不存在 -> 模型 spawn 失败
    #      -> controller_manager 联系不上 -> 控制器永远起不来。
    # `-x` 只比对进程名（comm），匹配不到上面这两种情况。
    #
    # robot_state_publisher 也一起清：上次 Gazebo 被强杀（比如关窗口、断电）时
    # 它会变成孤儿进程继续留着。gazebo_ros2_control 插件是按节点名去连它的，
    # 机器上有多个同名节点时插件可能连到旧的那个，读到过期的 URDF，
    # 用错误的初始位置去掰关节，模型会被物理引擎甩飞（表现为关节速度全是 NaN）。
    clean_gazebo = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -x gzserver 2>/dev/null; pkill -9 -x gzclient 2>/dev/null; '
             'for pid in $(pgrep -f "__node:=' + rsp_node + '"); do '
             '  [ "$pid" != "$$" ] && kill -9 "$pid" 2>/dev/null; '
             'done; '
             'sleep 1; exit 0'],
        output='screen',
    )

    xacro_file = os.path.join(pkg_description, 'urdf', 'my_robot.gazebo.urdf.xacro')
    # 单行化：gazebo_ros2_control 会把 robot_description 塞进 --param，
    # 多行 XML 会被 rcl 的 YAML 解析拒绝。去掉换行（保留属性内空格）即可。
    urdf_raw = subprocess.check_output(['xacro', xacro_file], text=True)
    urdf = ' '.join(urdf_raw.splitlines())
    robot_description = {'robot_description': urdf}

    # 把展开后的 URDF 落盘，方便离线用 `gz sdf -p` 看 Gazebo 实际转换出来的
    # SDF 到底长什么样（诊断用，不影响仿真）。
    urdf_dump = os.path.join(tempfile.gettempdir(), 'my_robot_gazebo.urdf')
    with open(urdf_dump, 'w') as f:
        f.write(urdf_raw)

    # URDF -> SDF：不能直接 spawn URDF。
    # sdformat 的转换器会写出 SDF 1.7 的 relative_to 写法，而 Gazebo Classic 11
    # 解析不了指向 link/joint 的 relative_to，结果所有连杆都被摆到模型原点
    # （只看得见底座、看不见机械臂）。urdf_to_sdf.py 把 link 的位姿换算成
    # 模型坐标系下的绝对位姿，退回 Gazebo Classic 支持的 SDF 1.6 语义。
    urdf_to_sdf = os.path.join(get_package_prefix('my_robot_description'),
                               'lib', 'my_robot_description', 'urdf_to_sdf.py')
    sdf_dump = os.path.join(tempfile.gettempdir(), 'my_robot_gazebo.sdf')
    # 用当前解释器跑脚本：colcon --symlink-install 装出来的是软链，
    # 不一定带可执行权限，直接 exec 会报 127。
    subprocess.check_call([sys.executable, urdf_to_sdf, urdf_dump, sdf_dump])

    # 1) Gazebo
    #    这里没有直接用 gazebo_ros 的 gazebo.launch.py：它不往下传 world 参数，
    #    而我们需要换成自己的世界文件（带 gazebo_ros_state 插件，
    #    这样才有 /gazebo/model_states、/gazebo/link_states 这些状态话题）。
    world_file = os.path.join(get_package_share_directory('my_robot_bringup'),
                              'worlds', 'empty_ros.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # 2) robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=rsp_node,
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 3) spawn_entity：把上面生成的 SDF 文件 spawn 进 Gazebo
    #    -z 0.01：底座碰撞盒底面原本正好落在 z=0 的地面上（零穿透、面面共面），
    #    这种退化接触会让 ODE 求解器解出 NaN。抬高 1cm 让它"落"下来而不是"贴"上去。
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', sdf_dump, '-entity', 'my_robot', '-z', '0.01'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 4) 激活控制器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 诊断：spawn 之后把 Gazebo 里真实存在的 link 及其位姿打到控制台。
    # （joint_states 里的 position 来自 ros2_control 的状态接口，
    #   link 塌掉/NaN 时它未必能反映 Gazebo 里的真实情况，这个才是。）
    diagnose = ExecuteProcess(
        cmd=['bash', '-c',
             'sleep 15; '
             'echo "=== [gazebo] 送进 Gazebo 的 SDF 里各 link 的绝对位姿 ==="; '
             'grep -A2 "<link name=" ' + sdf_dump + ' | grep -E "<link name=|<pose>" | '
             'paste - - | head -20; '
             'echo "=== [gazebo] Gazebo 里实际的 link 位姿 ==="; '
             'gz model --model-name my_robot --info 2>&1 || '
             'gz model -m my_robot -i 2>&1 || true; '
             'exit 0'],
        output='screen',
    )

    # 延迟 3 秒再启动 Gazebo：确保上面的清理动作确实跑完，
    # 即使 pkill 的匹配逻辑再出问题，也碰不到这一次的 gzserver。
    start = TimerAction(
        period=3.0,
        actions=[
            gzserver,
            gzclient,
            robot_state_publisher,
            spawn_entity,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
        ],
    )

    return LaunchDescription([
        clean_gazebo_arg,
        gui_arg,
        clean_gazebo,
        start,
        diagnose,
    ])
