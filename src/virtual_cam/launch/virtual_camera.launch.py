from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('virtual_cam'),
        'config',
        'virtual_camera.yaml'
    )

    return LaunchDescription([
        Node(
            package='virtual_cam',
            executable='virtual_camera_node',
            name='virtual_camera_node',
            parameters=[config],
            output='screen'
        )
    ])
