#include "yolo11_inference.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace as_training {

class YoloDetectorNode : public rclcpp::Node {
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  std::string model_path_;
  float conf_threshold_;
  float iou_threshold_;
  std::unique_ptr<YOLO11Detector> detector_;

  void image_callback(std::unique_ptr<sensor_msgs::msg::Image> msg) {
    cv::Mat frame(msg->height, msg->width, cv_bridge::getCvType(msg->encoding),
                  msg->data.data(), msg->step);
    drawDetections(frame, detector_->detect(frame));
    image_pub_->publish(std::move(msg));
  }

public:
  explicit YoloDetectorNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node(
            "yolo_detector_node",
            rclcpp::NodeOptions(options).use_intra_process_comms(true)),
        model_path_(this->declare_parameter<std::string>("model_path", "")),
        conf_threshold_(this->declare_parameter<float>("conf_threshold", 0.5)),
        iou_threshold_(this->declare_parameter<float>("iou_threshold", 0.45)),
        detector_(std::make_unique<YOLO11Detector>(model_path_, conf_threshold_,
                                                   iou_threshold_)) {
    RCLCPP_INFO(this->get_logger(), "Constructing yolo_detector_node... ");

    if (model_path_.empty()) {
      RCLCPP_INFO(this->get_logger(), "Invalid model path");
      throw std::runtime_error("Empty model path");
    }
    RCLCPP_INFO(this->get_logger(), "Valid model path");

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "raw_image", 10, [this](sensor_msgs::msg::Image::UniquePtr msg) {
          this->image_callback(std::move(msg));
        });
    image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("result_image", 10);
  }
};

} // namespace as_training
RCLCPP_COMPONENTS_REGISTER_NODE(as_training::YoloDetectorNode)