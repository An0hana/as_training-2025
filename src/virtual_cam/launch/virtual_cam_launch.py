import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('virtual_cam'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='virtual_cam', # 包名
            executable='virtual_cam_node_exe',
            name='virtual_cam_node', # 节点名
            parameters=[config],
            output='screen'
        )
    ])