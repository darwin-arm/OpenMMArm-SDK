#!/usr/bin/env python3
"""
OpenMMARM MoveIt 2 move_group 启动文件

启动 move_group 节点，加载 SRDF、运动学、关节限制、OMPL 等所有配置。
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


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
    moveit_config_pkg = get_package_share_directory('openmmarm_moveit_config')
    description_pkg = get_package_share_directory('openmmarm_description')

    # 使用 xacro 处理 robot.xacro
    xacro_file = os.path.join(description_pkg, 'urdf', 'robot.xacro')
    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 读取 SRDF
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'openmmarm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content
    }

    # 运动学配置
    kinematics_yaml = load_yaml('openmmarm_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }

    # 关节限制
    joint_limits_yaml = load_yaml('openmmarm_moveit_config', 'config/joint_limits.yaml')
    robot_description_planning = {
        'robot_description_planning': joint_limits_yaml
    }

    # OMPL 规划配置
    ompl_planning_yaml = load_yaml('openmmarm_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters':
                'default_planner_request_adapters/AddTimeOptimalParameterization '
                'default_planner_request_adapters/ResolveConstraintFrames '
                'default_planner_request_adapters/FixWorkspaceBounds '
                'default_planner_request_adapters/FixStartStateBounds '
                'default_planner_request_adapters/FixStartStateCollision '
                'default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    }
    if ompl_planning_yaml:
        ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # 轨迹执行配置
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    # 规划场景监控配置
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # 控制器配置
    moveit_controllers = load_yaml(
        'openmmarm_moveit_config', 'config/moveit_controllers.yaml'
    )
    moveit_controller_config = {}
    if moveit_controllers:
        moveit_controller_config = moveit_controllers

    # move_group 节点
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            planning_scene_monitor_parameters,
            moveit_controller_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用仿真时间',
        ),
        move_group_node,
    ])
