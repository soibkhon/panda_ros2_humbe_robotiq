# Merged launch file for Franka Emika Panda robot and Robotiq 2F-85 Gripper
# Licensed under the Apache License, Version 2.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # Franka Emika configuration
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    load_gripper_parameter_name = 'load_gripper'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    planner_parameter_name = 'planner'

    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    planner_ = LaunchConfiguration(planner_parameter_name)

    # Robotiq configuration
    description_pkg_share = FindPackageShare(package='robotiq_description').find('robotiq_description')
    default_model_path = os.path.join(description_pkg_share, "urdf", "robotiq_2f_85_gripper.urdf.xacro")
    default_rviz_config_path = os.path.join(description_pkg_share, "rviz", "view_urdf.rviz")

    # Declare common arguments
    args = [
        DeclareLaunchArgument(
            'model', default_value=default_model_path, description='Path to gripper URDF file'
        ),
        DeclareLaunchArgument(
            'rvizconfig', default_value=default_rviz_config_path, description='Path to RViz config file'
        ),
        DeclareLaunchArgument(
            'launch_rviz', default_value='false', description='Launch RViz?'
        ),
        DeclareLaunchArgument(
            robot_ip_parameter_name, description='Hostname or IP address of the robot.'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name, default_value='false', description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name, default_value='false', description='Use Franka Gripper'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name, default_value='false', description="Fake sensor commands"
        ),
        DeclareLaunchArgument(
            planner_parameter_name, default_value='ompl_interface/OMPLPlanner', description='Planner for arm control'
        ),
    ]

    # Franka Emika nodes
    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ', os.path.join(get_package_share_directory('franka_description'), 'robots', 'panda_arm.urdf.xacro'),
        ' hand:=', load_gripper, ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
        ' fake_sensor_commands:=', fake_sensor_commands
    ])
    robot_description_param = {'robot_description': robot_description_content}

    franka_nodes = [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[robot_description_param],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[robot_description_param],
        )
    ]

    # Robotiq nodes
    robotiq_nodes = [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description_param],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[robot_description_param],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_gripper_controller", "-c", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_activation_controller", "-c", "/controller_manager"],
        ),
        Node(
            package="robot_description",
            executable="gripper_control.py",
            output="screen"
        )
    ]

    return LaunchDescription(args + franka_nodes + robotiq_nodes)
