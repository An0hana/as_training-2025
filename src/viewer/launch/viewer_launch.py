import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('viewer'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        Node(
            package='viewer', # 包名
            executable='viewer_node_exe',
            name='viewer_node', # 节点名
            parameters=[config],
            output='screen'
        )
    ])