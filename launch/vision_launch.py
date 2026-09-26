from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='strawberry_vision',
            executable='camera_publisher',
            name='camera_publisher'
        ),
        Node(
            package='strawberry_vision',
            executable='vision_to_arm',
            name='vision_to_arm'
        ),
        Node(
            package='strawberry_vision',
            executable='robot_driver',
            name='robot_driver'
        ),
    ])
