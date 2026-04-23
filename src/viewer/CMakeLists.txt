cmake_minimum_required(VERSION 3.8)
project(viewer)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 寻找依赖(.cmake文件)
find_package(ament_cmake REQUIRED) # ROS依赖
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED) # 第三方依赖

# 编译动态库
add_library(viewer_node SHARED src/viewer_node.cpp)

# 链接ROS库
ament_target_dependencies(viewer_node # 查找 package.xml 中已注册的ROS依赖 
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
)

# 链接第三方库
target_link_libraries(viewer_node
  ${OpenCV_LIBS}
)

# 注册组件插件
rclcpp_components_register_node(viewer_node
  PLUGIN "as_training::ViewerNode"
  EXECUTABLE viewer_exe # 自动指定执行文件位置 install/viewer/lib/viewer/
)

# 共享执行文件夹 install/viewer/share/viewer
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 指定动态库位置 install/viewer/lib/
install(TARGETS 
  viewer_node
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