cmake_minimum_required(VERSION 3.10)
project(yolo11s_infer)

set(CMAKE_CXX_STANDARD 11)
set(OpenCV_DIR "/opt/opencv-4.7.0/share/opencv4")  # Update this path to your OpenCV build directory
find_package(OpenCV REQUIRED)

add_executable(infer main.cpp)
target_link_libraries(infer ${OpenCV_LIBS})
