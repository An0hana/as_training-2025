#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VirtualCameraNode : public rclcpp::Node
{
public:
  VirtualCameraNode() : Node("virtual_camera_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("video_path", "");
    this->declare_parameter<bool>("loop", true);
    this->declare_parameter<double>("frame_rate", 30.0);

    // Get parameters
    video_path_ = this->get_parameter("video_path").as_string();
    loop_ = this->get_parameter("loop").as_bool();
    frame_rate_ = this->get_parameter("frame_rate").as_double();

    if (video_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "video_path parameter is empty!");
      return;
    }

    // Open video file
    cap_.open(video_path_);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open video: %s", video_path_.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Opened video: %s", video_path_.c_str());

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/virtual_camera", 10);

    // Create timer based on frame rate
    auto period = std::chrono::duration<double>(1.0 / frame_rate_);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { this->timer_callback(); });
  }

private:
  void timer_callback()
  {
    cv::Mat frame;
    if (!cap_.read(frame)) {
      if (loop_) {
        // Reset to beginning of video
        cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
        if (!cap_.read(frame)) {
          RCLCPP_WARN(this->get_logger(), "Failed to read frame after reset");
          return;
        }
      } else {
        RCLCPP_INFO(this->get_logger(), "End of video reached");
        timer_->cancel();
        return;
      }
    }

    // Convert to ROS message
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();
    msg->header.frame_id = "virtual_camera";

    publisher_->publish(*msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  std::string video_path_;
  bool loop_;
  double frame_rate_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualCameraNode>());
  rclcpp::shutdown();
  return 0;
}
