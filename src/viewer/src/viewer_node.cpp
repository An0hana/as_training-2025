#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace as_training {

class ViewerNode : public rclcpp::Node {
public:
  explicit ViewerNode(const rclcpp::NodeOptions &options)
      : Node("viewer_node", options) {
     
    RCLCPP_INFO(this->get_logger(), "Constructing node...");
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "result_image", 
        10, 
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            try {
                cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8"); // 获取底层数据指针，格式转换
                cv::imshow("Stream Viewer", cv_ptr->image); // 渲染
                cv::waitKey(1); // 刷新 GUI 线程
                
            } catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "cv_bridge 异常: %s", e.what());
            }
        });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

} // namespace as_training
RCLCPP_COMPONENTS_REGISTER_NODE(as_training::ViewerNode)