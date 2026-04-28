#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace as_training {
class LidarPerceptionNode : public rclcpp::Node {
public:
    explicit LidarPerceptionNode(const rclcpp::NodeOptions &options) 
        : Node("lidar_perception_node", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        RCPCPP_INFO(this->get_logger(), "Constructing lidar_perception_node...")
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar/pointcloud", 10,
            [this](sensor_msgs::msg::PointCloud2::UniquePtr msg){this->cloud_callback(std::move(msg));}
        );
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/filtered_points", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/cone_markers", 10);
        RCLCPP_INFO(this->get_logger(), "Done with constructing lidar_perception_node.")
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    void cloud_callback(std::unique_ptr<sensor_msgs::msg::PointCloud2> msg) {
        
    }
};
} // namespace as_training

RCLCPP_COMPONENTS_REGISTER_NODE(as_training::LidarPerceptionNode)