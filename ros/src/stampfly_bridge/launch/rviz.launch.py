"""
rviz.launch.py - Launch StampFly bridge with RViz2 visualization

Usage:
    ros2 launch stampfly_bridge rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('stampfly_bridge')

    # RViz config file
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='192.168.4.1',
        description='StampFly IP address'
    )

    # Include bridge launch
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'bridge.launch.py')
        ),
        launch_arguments={
            'host': LaunchConfiguration('host'),
        }.items(),
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        host_arg,
        bridge_launch,
        rviz_node,
    ])
