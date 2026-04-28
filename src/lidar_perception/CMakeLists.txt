cmake_minimum_required(VERSION 3.8)
project(lidar_perception)
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED) # ROS依赖
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters) # 第三方依赖

add_library(lidar_perception_node SHARED src/lidar_perception_node.cpp) # 编译动态库

ament_target_dependencies(lidar_perception_node # 链接ROS库
  rclcpp
  rclcpp_components
  sensor_msgs
  pcl_conversions
  visualization_msgs
)

target_include_directories(lidar_perception_node PUBLIC # 链接第三方库
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar_perception_node
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(lidar_perception_node # 注册组件插件
  PLUGIN "as_training::LidarPerceptionNode"
  EXECUTABLE lidar_perception_node_exe
)

install(DIRECTORY # 共享执行文件夹
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS # 指定动态库位置
  lidar_perception_node
  DESTINATION lib)

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
