#!/usr/bin/env python3
"""
OpenMMARM 仿真模式启动文件

启动完整的仿真环境：
  1. openmmarm_controller（ROS 通信模式 + 仿真）
  2. openmmarm_hw（ros2_control 硬件接口）
  3. move_group（MoveIt 运动规划）
  4. robot_state_publisher（TF 发布）
  5. rviz2（可选）
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    """从包中加载 YAML 文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception:
        return None


def generate_launch_description():
    # 包路径
    description_pkg = get_package_share_directory('openmmarm_description')
    moveit_config_pkg = get_package_share_directory('openmmarm_moveit_config')

    # Launch 参数
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_moveit = LaunchConfiguration('use_moveit')

    # 使用 xacro 处理 robot.xacro
    xacro_file = os.path.join(description_pkg, 'urdf', 'robot.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time},
        ],
    )

    # openmmarm_controller 节点（ROS 模式，模拟真实硬件）
    # 注意：在 sim 模式下，我们启动 controller 并让它连接到 loopback
    # 这里的 controller 是被控对象，不是 moveit 的 controller
    controller_node = Node(
        package='openmmarm_controller',
        executable='openmmarm_ctrl',
        name='openmmarm_controller',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'communication': 'SIM'},
            {'sim.model_path': LaunchConfiguration('sim_model_path')},
        ],
    )

    # openmmarm_hw 启动（ros2_control 硬件接口）
    # 延时 2 秒启动，等待 controller 就绪
    hw_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('openmmarm_hw'),
                        'launch',
                        'openmmarm_hw.launch.py',
                    ])
                ]),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            )
        ]
    )

    # MoveIt move_group 启动
    # 延时 5 秒启动，等待 hw 就绪
    moveit_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('openmmarm_moveit_config'),
                        'launch',
                        'move_group.launch.py',
                    ])
                ]),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
                condition=IfCondition(use_moveit),
            )
        ]
    )

    # 加载 SRDF 和 kinematics 配置（RViz MotionPlanning 插件需要）
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'openmmarm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('openmmarm_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}

    # RViz2 节点
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='启动 RViz2',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间',
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='true',
            description='启动 MoveIt move_group',
        ),
        DeclareLaunchArgument(
            'sim_model_path',
            default_value='',
            description='MuJoCo 模型文件路径 (留空则自动使用 openmmarm_description 中的 URDF)',
        ),
        controller_node,
        robot_state_publisher_node,
        hw_launch,
        moveit_launch,
        rviz_node,
    ])
