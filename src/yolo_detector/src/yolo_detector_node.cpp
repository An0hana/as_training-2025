#include "yolo11_inference.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

namespace as_training {

class YoloDetectorNode : public rclcpp::Node {

};

}
RCLCPP_COMPONENTS_REGISTER_NODE(as_training::YoloDetectorNode)