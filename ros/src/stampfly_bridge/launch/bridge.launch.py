"""
bridge.launch.py - Launch StampFly ROS2 bridge node

Usage:
    ros2 launch stampfly_bridge bridge.launch.py
    ros2 launch stampfly_bridge bridge.launch.py host:=192.168.4.1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    # ローンチ引数宣言
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='192.168.4.1',
        description='StampFly IP address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='80',
        description='WebSocket port'
    )

    path_arg = DeclareLaunchArgument(
        'path',
        default_value='/ws',
        description='WebSocket path'
    )

    auto_reconnect_arg = DeclareLaunchArgument(
        'auto_reconnect',
        default_value='true',
        description='Auto-reconnect on disconnect'
    )

    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF transforms'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame ID'
    )

    imu_frame_arg = DeclareLaunchArgument(
        'imu_frame',
        default_value='imu_link',
        description='IMU frame ID'
    )

    # Control parameters (Phase 2)
    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='false',
        description='Enable control (ROS2 -> StampFly)'
    )

    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='50.0',
        description='Control send rate (Hz)'
    )

    max_throttle_arg = DeclareLaunchArgument(
        'max_throttle',
        default_value='0.8',
        description='Maximum throttle (safety limit)'
    )

    # Bridge node
    bridge_node = Node(
        package='stampfly_bridge',
        executable='bridge_node',
        name='stampfly_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'path': LaunchConfiguration('path'),
            'auto_reconnect': LaunchConfiguration('auto_reconnect'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'imu_frame': LaunchConfiguration('imu_frame'),
            'enable_control': LaunchConfiguration('enable_control'),
            'control_rate': LaunchConfiguration('control_rate'),
            'max_throttle': LaunchConfiguration('max_throttle'),
        }],
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        path_arg,
        auto_reconnect_arg,
        publish_tf_arg,
        odom_frame_arg,
        base_frame_arg,
        imu_frame_arg,
        enable_control_arg,
        control_rate_arg,
        max_throttle_arg,
        bridge_node,
    ])
