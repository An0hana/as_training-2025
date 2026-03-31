cmake_minimum_required(VERSION 3.10)
project(yolo_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
# 查找 ONNX Runtime
set(ONNXRUNTIME_DIR "/opt/onnxruntime")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# 引入ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 创建可执行文件
add_executable(yolo_detector_node src/yolo_detector_node.cpp)

# 链接ROS2库
ament_target_dependencies(yolo_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
)

# 链接第三方库
target_link_libraries(yolo_detector_node
    ${OpenCV_LIBS}
    onnxruntime
)

install(TARGETS yolo_detector_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()