from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='uart_node', executable='uart_sender', name='uart_sender', output='screen'),
        Node(package='vision_node', executable='vision_node', name='vision_node', output='screen'),
    ])
