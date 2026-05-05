#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>

namespace as_training {

class VirtualCamNode : public rclcpp::Node {
private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  std::string video_path_;
  double video_fps_;

  void timer_callback(cv::Mat &frame) { // lambda 捕获内存复用
    auto start_time = std::chrono::steady_clock::now();
    cap_.read(frame);
    if (frame.empty()) {
      cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
      cap_.read(frame);
    }
    // cv::resize(frame, buffer, cv::Size(960, 960));

    auto msg = std::make_unique<sensor_msgs::msg::Image>(); // 独占指针
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_link";
    cv_bridge::CvImage(header, "bgr8", frame).toImageMsg(*msg); // cv_bridge 拷贝打包
    image_pub_->publish(std::move(msg)); // 移交所有权

    auto end_time = std::chrono::steady_clock::now();
    double cost_ms =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();
    RCLCPP_INFO(this->get_logger(), "%.2f ms", cost_ms);
  }

public:
  explicit VirtualCamNode(const rclcpp::NodeOptions &options)
      : rclcpp::Node(
            "virtual_cam_node",
            rclcpp::NodeOptions(options).use_intra_process_comms(true)) { // 开启进程间通信
    RCLCPP_INFO(this->get_logger(), "Constructing node...");

    this->declare_parameter<std::string>("video_path", ""); // 路径可做参数传递
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
        std::function<void()>(
            [this, frame = cv::Mat()]() mutable { // lambda 回调函数
              this->timer_callback(frame);
            }));

    image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("raw_image", 10);
  }
};

} // namespace as_training
RCLCPP_COMPONENTS_REGISTER_NODE(as_training::VirtualCamNode)