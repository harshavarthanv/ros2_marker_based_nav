from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='group16',  # Replace with your package name
            executable='listen',  # Replace with your executable name
            name='listen',
            # Replace with the path to your param.yaml
            parameters=[{'/home/anbarasan/assignments/rwa3_ws/src/group16/config/params.yaml'}]
        )
    ])
