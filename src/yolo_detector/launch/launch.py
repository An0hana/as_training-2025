import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    yolo_pkg_dir = get_package_share_directory('yolo_detector')
    model_path = os.path.join(yolo_pkg_dir, 'weight', 'best.onnx')
    video_file_path = '/workspace/as_training-2025/data/video/video.mp4'

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),

        ComposableNodeContainer(
            name='viewer_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                
                ComposableNode(
                    package='virtual_cam',
                    plugin='as_training::VirtualCamNode',
                    parameters=[{"video_path": video_file_path}],
                    name='virtual_cam_node',
                    extra_arguments=[{'use_intra_process_comms': True}] # 开启零拷贝
                ),
                
                ComposableNode(
                    package='yolo_detector',
                    plugin='as_training::YoloDetectorNode',
                    name='yolo_detector_node',
                    parameters=[{"model_path": model_path}], 
                    extra_arguments=[{'use_intra_process_comms': True}] # 开启零拷贝
                ),

            ],
            output='screen',
        )
    ])