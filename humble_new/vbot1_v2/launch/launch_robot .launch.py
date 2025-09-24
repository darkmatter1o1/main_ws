import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'vbot1_v2'

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    # Process URDF/Xacro
    xacro_file = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=', use_ros2_control,
        ' sim_mode:=', use_sim_time
    ])
    robot_description = {'robot_description': robot_description_content}

    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Controller manager node
    controller_params_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'my_controllers.yaml'
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
        output='screen'
    )

    # Spawners for controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
        output='screen'
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
        output='screen'
    )

    # Add delays to ensure proper startup order
    delayed_controller_manager = TimerAction(
        period=3.0,
        actions=[controller_manager_node]
    )

    delayed_diff_drive_spawner = TimerAction(
        period=5.0,
        actions=[diff_drive_spawner]
    )

    delayed_joint_broad_spawner = TimerAction(
        period=6.0,
        actions=[joint_broad_spawner]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_ros2_control', default_value='true'),
        robot_state_publisher_node,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner
    ])
