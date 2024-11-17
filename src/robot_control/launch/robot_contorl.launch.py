from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robot_control', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen'),
        Node(package='robot_control', executable='robot_monitor',
            name='robot_monitor', output='screen'),
        Node(package='robot_control', executable='robot_mover',
            name='robot_mover', output='screen')
    ])
