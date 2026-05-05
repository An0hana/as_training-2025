import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    pkg_dir = get_package_share_directory('yolo_detector') # 获取 install/yolo_detetor/share/yolo_detector
    config = os.path.join( # 路径拼接
        pkg_dir,
        'config',
        'params.yaml'
    )
    model_path = os.path.join(
        pkg_dir,
        'weight',
        'best.onnx'
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        Node(
            package='yolo_detector', # 包名
            executable='yolo_detector_node_exe', # 可执行文件名
            name='yolo_detector_node', # 节点名
            parameters=[config, {"model_path": model_path}], # 参数列表
            output='screen'
        )
    ])