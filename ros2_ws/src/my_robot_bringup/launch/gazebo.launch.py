import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """在 Gazebo 真实物理世界里启动 my_robot 机械臂。

    启动顺序/职责：
      0) 清理残留的 gzserver/gzclient —— 上一次运行若没正常退出，新实例会因
         11345 端口被占用而起不来，此时屏幕上看到的还是旧 Gazebo 窗口，
         容易误判为"模型没更新"。
      1) gazebo          —— 启动 Gazebo 世界（默认空世界，带 GUI）
      2) robot_state_publisher —— 处理 my_robot.gazebo.urdf.xacro，发布 TF 和
                            /robot_description 话题（spawn_entity 从中读取模型）
      3) spawn_entity    —— 把模型 spawn 进 Gazebo；模型里的 gazebo_ros2_control
                            插件会创建 controller_manager 并加载控制器参数
      4) spawner x2      —— 激活 joint_state_broadcaster / arm_controller

    控制方式（仿真起来后）：
      ros2 topic pub /arm_controller/joint_trajectory \
        trajectory_msgs/msg/JointTrajectory "..." --once
    """
    pkg_description = get_package_share_directory('my_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = True

    clean_gazebo_arg = DeclareLaunchArgument(
        'clean_gazebo',
        default_value='true',
        description='Kill leftover gzserver/gzclient processes before starting.'
    )
    clean_gazebo = ExecuteProcess(
        cmd=['bash', '-c',
             'pkill -9 -f gzserver 2>/dev/null; pkill -9 -f gzclient 2>/dev/null; '
             'sleep 1; exit 0'],
        output='screen',
    )

    xacro_file = os.path.join(pkg_description, 'urdf', 'my_robot.gazebo.urdf.xacro')
    # 单行化：gazebo_ros2_control 会把 robot_description 塞进 --param，
    # 多行 XML 会被 rcl 的 YAML 解析拒绝。去掉换行（保留属性内空格）即可。
    urdf = subprocess.check_output(['xacro', xacro_file], text=True)
    urdf = ' '.join(urdf.splitlines())
    robot_description = {'robot_description': urdf}

    # 1) Gazebo（空世界 + GUI）
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        )
    )

    # 2) robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 3) spawn_entity：从 /robot_description 话题读取并 spawn 模型
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
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

    return LaunchDescription([
        clean_gazebo_arg,
        clean_gazebo,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
