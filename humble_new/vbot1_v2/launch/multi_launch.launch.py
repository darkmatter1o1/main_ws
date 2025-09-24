from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    articubot_dir = get_package_share_directory('vbot1_v2')
    pkg_qt_gui_ros2 = get_package_share_directory('qt_gui_ros2')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map', default_value=os.path.join(articubot_dir, 'worlds', 'newmap.yaml')),

        # Launch robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(articubot_dir, 'launch', 'rplidar.launch.py')
            )
        ),

        # Delay 2 seconds before launching RPLiDAR


        TimerAction(
            period=25.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='amcl_pose',
                    name='imu_publisher_node',
                    output='screen',
                )
            ]
        ),



        TimerAction(
            period=60.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='battery',
                    name='battery_node',
                    output='screen',
                )
            ]
        ),

        TimerAction(
            period=70.0,
            actions=[
                Node(
                     package='my_imu_publisher',
                    executable='imu_publisher',
                    name='obstacle_node',
                    output='screen',
                )
            ]
        ),

    ])
