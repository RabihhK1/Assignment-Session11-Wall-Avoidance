import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('obstacle_avoidance'),
        'config',
        'config.yaml'
        )

    return LaunchDescription([
        Node(
            package='obstacle_avoidance',
            namespace='avoidance_namespace',
            executable='avoidance',
            name='avoidance',
            parameters=[config]
        ),
    ])
