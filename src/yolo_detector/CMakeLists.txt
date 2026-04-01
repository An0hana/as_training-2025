cmake_minimum_required(VERSION 3.8)
project(yolo_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

# 第三方依赖
find_package(OpenCV REQUIRED)
set(ONNXRUNTIME_DIR "/opt/onnxruntime")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# 编译动态库
add_library(yolo_detector_node SHARED src/yolo_detector_node.cpp)

# 链接ROS库
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

# 注册组件插件
rclcpp_components_register_node(yolo_detector_node
  PLUGIN "as_training::YoloDetectorNode"
  EXECUTABLE yolo_detector_node_exe
)

install(TARGETS yolo_detector_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()