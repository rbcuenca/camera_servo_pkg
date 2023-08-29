from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_servo_pkg',
            executable='camera_servo',
            output='screen'),
    ])