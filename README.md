# AS_Training-2025

1. 任务一：使用 virtual_cam 节点作为虚拟相机播放视频到指定话题（该节点不能被修改），创建节点，将weight/onnx_infer.cpp中的YOLO11Detector类作为成员变量，对虚拟相机话题进行推理，使用零拷贝和组件节点。
效果参考：
2. 任务二：播放bag,使用地面分割算法和点云聚类算法对锥桶进行聚类
效果参考：




参考：
配置 onnxruntime
选择下载到并解压到指定目录，并设置环境变量

export ONNXRUNTIME_DIR="/home/as/onnxruntime"
export LD_LIBRARY_PATH=/home/as/onnxruntime/lib:$LD_LIBRARY_PATH

https://github.com/microsoft/onnxruntime/releases/

地面分割算法参考：
https://github.com/url-kaist



