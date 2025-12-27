cmake_minimum_required(VERSION 3.10)
project(yolo11_onnx_infer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 查找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
# 查找 ONNX Runtime
# 请根据实际安装路径修改 ONNXRUNTIME_DIR
set(ONNXRUNTIME_DIR "/home/as/onnxruntime")
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

# 创建可执行文件
add_executable(yolo11_infer onnx_infer.cpp)

# 链接库
target_link_libraries(yolo11_infer
    ${OpenCV_LIBS}
    onnxruntime
)
