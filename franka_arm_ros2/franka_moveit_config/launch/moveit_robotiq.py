#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

import os
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
    # Common parameters
    robot_ip_parameter_name = 'robot_ip'
    use_fake_hardware_parameter_name = 'use_fake_hardware'
    load_gripper_parameter_name = 'load_gripper'
    fake_sensor_commands_parameter_name = 'fake_sensor_commands'
    planner_parameter_name = 'planner'

    # Launch configurations
    robot_ip = LaunchConfiguration(robot_ip_parameter_name)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter_name)
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter_name)
    planner_ = LaunchConfiguration(planner_parameter_name)

    # Get package paths
    robotiq_description_pkg_share = get_package_share_directory("robotiq_description")
    
    default_robotiq_model_path = os.path.join(
        robotiq_description_pkg_share, "urdf", "robotiq_2f_85_gripper.urdf.xacro"
    )
    default_robotiq_rviz_config_path = os.path.join(
        robotiq_description_pkg_share, "rviz", "view_urdf.rviz"
    )

    # Command-line arguments
    args = [
        DeclareLaunchArgument(
            'db', default_value='False', description='Database flag'
        ),
        DeclareLaunchArgument(
            robot_ip_parameter_name,
            description='Hostname or IP address of the robot.'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter_name,
            default_value='false',
            description='Use fake hardware'
        ),
        DeclareLaunchArgument(
            planner_parameter_name,
            default_value='ompl_interface/OMPLPlanner',
            description='Choose planner to be used for arm control'
        ),
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='false',
            description='Use Franka Gripper as an end-effector'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter_name,
            default_value='false',
            description="Fake sensor commands. Only valid when use_fake_hardware is true"
        ),
        DeclareLaunchArgument(
            name="robotiq_model",
            default_value=default_robotiq_model_path,
            description="Absolute path to Robotiq gripper URDF file",
        ),
        DeclareLaunchArgument(
            name="robotiq_rvizconfig",
            default_value=default_robotiq_rviz_config_path,
            description="Absolute path to Robotiq rviz config file",
        ),
        DeclareLaunchArgument(
            name="launch_robotiq_rviz", 
            default_value="false", 
            description="Launch RViz for Robotiq?"
        )
    ]

    # Franka robot description
    franka_xacro_file = os.path.join(get_package_share_directory('franka_description'), 'robots',
                                    'panda_arm.urdf.xacro')
    robot_description_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper,
         ' robot_ip:=', robot_ip, ' use_fake_hardware:=', use_fake_hardware,
         ' fake_sensor_commands:=', fake_sensor_commands])

    robot_description = {'robot_description': robot_description_config}

    # Robotiq gripper description
    robotiq_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            LaunchConfiguration("robotiq_model"),
            " ",
            "use_fake_hardware:=false",
        ]
    )
    robotiq_description_param = {
        "robot_description": ParameterValue(
            robotiq_description_content, value_type=str
        )
    }

    # Load configurations and controllers
    franka_semantic_xacro_file = os.path.join(get_package_share_directory('franka_moveit_config'),
                                             'srdf',
                                             'panda_arm.srdf.xacro')
    robot_description_semantic_config = Command(
        [FindExecutable(name='xacro'), ' ', franka_semantic_xacro_file, ' hand:=', load_gripper]
    )
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_config
    }

    kinematics_yaml = load_yaml(
        'franka_moveit_config', 'config/kinematics.yaml'
    )

    # Planning configurations
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': '''default_planner_request_adapters/AddTimeOptimalParameterization 
                                 default_planner_request_adapters/ResolveConstraintFrames 
                                 default_planner_request_adapters/FixWorkspaceBounds 
                                 default_planner_request_adapters/FixStartStateBounds 
                                 default_planner_request_adapters/FixStartStateCollision 
                                 default_planner_request_adapters/FixStartStatePathConstraints''',
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        'franka_moveit_config', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['move_group'].update(ompl_planning_yaml)

    # Trajectory Execution Functionality
    moveit_simple_controllers_yaml = load_yaml(
        'franka_moveit_config', 'config/panda_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # Robotiq controller configurations
    update_rate_config_file = PathJoinSubstitution(
        [
            robotiq_description_pkg_share,
            "config",
            "robotiq_update_rate.yaml",
        ]
    )

    robotiq_controllers_file = "robotiq_controllers.yaml"
    initial_joint_controllers = PathJoinSubstitution(
        [robotiq_description_pkg_share, "config", robotiq_controllers_file]
    )

    # Nodes
    nodes = [
        # Franka nodes
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                robot_description,
                robot_description_semantic,
                kinematics_yaml,
                ompl_planning_pipeline_config,
                trajectory_execution,
                moveit_controllers,
                planning_scene_monitor_parameters,
                {"publish_robot_description_semantic": True},
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            parameters=[robot_description],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, update_rate_config_file, initial_joint_controllers],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
            on_exit=Shutdown(),
        ),

        # Load controllers
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["panda_arm_controller", "-c", "/controller_manager"],
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
    ]

    # Optional RViz nodes
    rviz_base = os.path.join(get_package_share_directory('franka_moveit_config'), 'rviz')
    rviz_full_config = os.path.join(rviz_base, 'moveit.rviz')
    
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
            arguments=['-d', rviz_full_config],
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_planning_pipeline_config,
                kinematics_yaml,
            ],
        )
    )

    return LaunchDescription(args + nodes)