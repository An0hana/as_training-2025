#include <chrono>
#include <opencv2/opencv.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

namespace as_training {

class VirtualCamNode : public rclcpp::Node {
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  std::string video_path_;
  double video_fps_;

  void timer_callback() {
    auto start_time = std::chrono::steady_clock::now();
    cv::Mat frame;
    sensor_msgs::msg::Image msg;
    size_t data_size;

    cap_.read(frame);
    if (frame.empty()) {
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      cap_.read(frame);
    }
    cv::resize(frame, frame, cv::Size(640, 360));

    data_size = frame.step * frame.rows;
    msg.data.assign(frame.data, frame.data + data_size);

    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_link";
    msg.height = frame.rows;
    msg.width = frame.cols;
    msg.encoding = "bgr8";
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);

    image_pub_->publish(msg);

    auto end_time = std::chrono::steady_clock::now();
    double cost_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();
    RCLCPP_INFO(this->get_logger(), "%.2f ms", cost_ms);
  }

public:
  explicit VirtualCamNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node("virtual_cam_node", options) { // 节点名
    RCLCPP_INFO(this->get_logger(), "Constructing node...");

    this->declare_parameter<std::string>("video_path", "");
    video_path_ = this->get_parameter("video_path").as_string();

    cap_.open(video_path_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Invalid video path");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "valid video path");

    video_fps_ = cap_.get(cv::CAP_PROP_FPS);
    // RCLCPP_INFO(this->get_logger(), "%f", video_fps_);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double, std::milli>(1000.0 / video_fps_),
        [this]() { this->timer_callback(); });

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "raw_image", rclcpp::SensorDataQoS());
  }
};

} // namespace as_training
RCLCPP_COMPONENTS_REGISTER_NODE(as_training::VirtualCamNode)