#pragma once
#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

struct Detection {
  cv::Rect box;
  float confidence;
  int classId;
};

class YOLO11Detector {
public:
  YOLO11Detector(const std::string &modelPath, float confThreshold = 0.35f,
                 float iouThreshold = 0.45f);
  std::vector<Detection> detect(const cv::Mat &image);

private:
  Ort::Env env_{nullptr};
  Ort::Session session_{nullptr};
  std::string inputName_;
  std::string outputName_;
  int64_t inputWidth_ = 960;
  int64_t inputHeight_ = 960;
  float confThreshold_;
  float iouThreshold_;

  void preprocess(const cv::Mat &image, cv::Mat &blob);
  std::vector<Detection> postprocess(float *output,
                                     const std::vector<int64_t> &shape,
                                     int origWidth, int origHeight);
};

void drawDetections(cv::Mat &image, const std::vector<Detection> &detections);