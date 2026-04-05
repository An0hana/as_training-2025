find_path(ONNXRUNTIME_INCLUDE_DIRS # 寻找头文件路径
    NAMES onnxruntime_cxx_api.h
    PATH_SUFFIXES include
)

find_library(ONNXRUNTIME_LIBRARIES # 寻找库文件
    NAMES onnxruntime
    PATHS_SUFFIXES lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(onnxruntime
    DEFAULT_MSG
    ONNXRUNTIME_INCLUDE_DIRS
    ONNXRUNTIME_LIBRARIES
)

if(onnxruntime_FOUND AND NOT TARGET onnxruntime::onnxruntime) # 创建命名空间
    add_library(onnxruntime::onnxruntime UNKNOWN IMPORTED)
    set_target_properties(onnxruntime::onnxruntime PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${ONNXRUNTIME_INCLUDE_DIRS}"
        IMPORTED_LOCATION "${ONNXRUNTIME_LIBRARIES}"
    )
endif()
