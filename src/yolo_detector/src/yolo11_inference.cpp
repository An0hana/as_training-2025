#include "yolo11_inference.hpp"
#include <algorithm>
#include <array>
#include <cstring>
#include <iostream>

YOLO11Detector::YOLO11Detector(const std::string &modelPath,
                               float confThreshold, float iouThreshold)
    : confThreshold_(confThreshold), iouThreshold_(iouThreshold) {

  // 初始化 ONNX Runtime
  env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YOLO11");
  Ort::SessionOptions sessionOptions;
  sessionOptions.SetIntraOpNumThreads(4);
  sessionOptions.SetGraphOptimizationLevel(
      GraphOptimizationLevel::ORT_ENABLE_ALL);

  session_ = Ort::Session(env_, modelPath.c_str(), sessionOptions);

  // 获取输入输出信息
  Ort::AllocatorWithDefaultOptions allocator;

  // 输入信息
  auto inputName = session_.GetInputNameAllocated(0, allocator);
  inputName_ = inputName.get();
  auto inputShape =
      session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
  inputHeight_ = inputShape[2];
  inputWidth_ = inputShape[3];

  // 输出信息
  auto outputName = session_.GetOutputNameAllocated(0, allocator);
  outputName_ = outputName.get();

  std::cout << "load success!" << std::endl;
  std::cout << "input size: " << inputWidth_ << "x" << inputHeight_
            << std::endl;
}

std::vector<Detection> YOLO11Detector::detect(const cv::Mat &image) {
  cv::Mat blob;
  preprocess(image, blob);

  // 准备输入张量
  std::vector<int64_t> inputShape = {1, 3, inputHeight_, inputWidth_};
  size_t inputTensorSize = 1 * 3 * inputHeight_ * inputWidth_;

  std::vector<float> inputTensorValues(inputTensorSize);
  std::memcpy(inputTensorValues.data(), blob.data,
              inputTensorSize * sizeof(float));

  auto memoryInfo =
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
  Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
      memoryInfo, inputTensorValues.data(), inputTensorSize, inputShape.data(),
      inputShape.size());

  // 运行推理
  std::array<const char *, 1> inputNames = {inputName_.c_str()};
  std::array<const char *, 1> outputNames = {outputName_.c_str()};

  auto outputTensors = session_.Run(Ort::RunOptions{nullptr}, inputNames.data(),
                                    &inputTensor, 1, outputNames.data(), 1);

  // 获取输出
  auto *outputData = outputTensors[0].GetTensorMutableData<float>();
  auto outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();

  // 后处理
  return postprocess(outputData, outputShape, image.cols, image.rows);
}

void YOLO11Detector::preprocess(const cv::Mat &image, cv::Mat &blob) {
  cv::Mat resized;
  cv::resize(image, resized, cv::Size(inputWidth_, inputHeight_));

  // BGR -> RGB, 归一化到 [0, 1]
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(rgb, CV_32F, 1.0 / 255.0);

  // HWC -> CHW
  std::vector<cv::Mat> channels(3);
  cv::split(rgb, channels);

  blob = cv::Mat(
      3, std::vector<int>{3, (int)inputHeight_, (int)inputWidth_}.data(),
      CV_32F);
  for (int c = 0; c < 3; c++) {
    std::memcpy(blob.data + c * inputHeight_ * inputWidth_ * sizeof(float),
                channels[c].data, inputHeight_ * inputWidth_ * sizeof(float));
  }
}

std::vector<Detection>
YOLO11Detector::postprocess(float *output, const std::vector<int64_t> &shape,
                            int origWidth, int origHeight) {
  std::vector<Detection> detections;
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  std::vector<int> classIds;

  // YOLO11 输出格式: [1, 84, 8400] -> [1, 4+num_classes, num_predictions]
  // 需要转置为 [8400, 84]
  int numClasses = shape[1] - 4; // 通常是80个类别
  int numPredictions = shape[2];

  float xFactor = (float)origWidth / inputWidth_;
  float yFactor = (float)origHeight / inputHeight_;

  for (int i = 0; i < numPredictions; i++) {
    // 获取边界框坐标 (cx, cy, w, h)
    float cx = output[0 * numPredictions + i];
    float cy = output[1 * numPredictions + i];
    float w = output[2 * numPredictions + i];
    float h = output[3 * numPredictions + i];

    // 找到最大类别置信度
    float maxClassConf = 0;
    int maxClassId = 0;
    for (int c = 0; c < numClasses; c++) {
      float classConf = output[(4 + c) * numPredictions + i];
      if (classConf > maxClassConf) {
        maxClassConf = classConf;
        maxClassId = c;
      }
    }

    if (maxClassConf >= confThreshold_) {
      // 转换为 xyxy 格式并缩放到原始图像尺寸
      int x1 = static_cast<int>((cx - w / 2) * xFactor);
      int y1 = static_cast<int>((cy - h / 2) * yFactor);
      int x2 = static_cast<int>((cx + w / 2) * xFactor);
      int y2 = static_cast<int>((cy + h / 2) * yFactor);

      boxes.emplace_back(x1, y1, x2 - x1, y2 - y1);
      confidences.emplace_back(maxClassConf);
      classIds.emplace_back(maxClassId);
    }
  }

  // NMS
  std::vector<int> indices;
  cv::dnn::NMSBoxes(boxes, confidences, confThreshold_, iouThreshold_, indices);

  for (int idx : indices) {
    Detection det;
    det.box = boxes[idx];
    det.confidence = confidences[idx];
    det.classId = classIds[idx];
    detections.emplace_back(det);
  }

  return detections;
}

// 在图像上绘制检测结果
void drawDetections(cv::Mat &image, const std::vector<Detection> &detections) {
  // 锥桶类别名称 (3类)
  static const std::vector<std::string> classNames = {"red_cone", "blue_cone",
                                                      "yellow_cone"};
  // 对应颜色 (BGR格式)
  static const std::vector<cv::Scalar> classColors = {
      cv::Scalar(0, 0, 255),  // 黄色
      cv::Scalar(255, 0, 0),  // 蓝色
      cv::Scalar(0, 165, 255) // 橙色
  };

  for (const auto &det : detections) {
    // 根据类别选择颜色
    cv::Scalar color = (static_cast<size_t>(det.classId) < classColors.size())
                           ? classColors[det.classId]
                           : cv::Scalar(0, 255, 0);

    // 绘制边界框
    cv::rectangle(image, det.box, color, 2);

    // 准备标签文本
    std::string label;
    if (static_cast<size_t>(det.classId) < classNames.size()) {
      label = classNames[det.classId];
    } else {
      label = "class" + std::to_string(det.classId);
    }
    label +=
        ": " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";

    // 绘制标签背景
    int baseLine;
    cv::Size labelSize =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    int top = std::max(det.box.y, labelSize.height);
    cv::rectangle(image, cv::Point(det.box.x, top - labelSize.height - 5),
                  cv::Point(det.box.x + labelSize.width, top), color,
                  cv::FILLED);

    // 绘制标签文本
    cv::putText(image, label, cv::Point(det.box.x, top - 3),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  }
}
