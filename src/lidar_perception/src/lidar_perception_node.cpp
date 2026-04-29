#include <queue>
#include <chrono>
#include <vector>
#include <cmath>
#include <algorithm>

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

namespace as_training {

class LidarPerceptionNode : public rclcpp::Node {
public:
    explicit LidarPerceptionNode(const rclcpp::NodeOptions &options) 
        : Node("lidar_perception_node", rclcpp::NodeOptions(options).use_intra_process_comms(true))
    {
        RCLCPP_INFO(this->get_logger(), "Architecting L4 Perception Pipeline...");

        pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_roi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_filtered_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_obstacles_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_near_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        cloud_far_.reset(new pcl::PointCloud<pcl::PointXYZI>);
        
        inliers_.reset(new pcl::PointIndices);
        coefficients_.reset(new pcl::ModelCoefficients);
        kdtree_.reset(new pcl::search::KdTree<pcl::PointXYZI>);

        load_params();

        pass_x_.setFilterFieldName("x"); pass_x_.setFilterLimits(params_.roi_x_min, params_.roi_x_max);
        pass_y_.setFilterFieldName("y"); pass_y_.setFilterLimits(params_.roi_y_min, params_.roi_y_max);
        pass_z_.setFilterFieldName("z"); pass_z_.setFilterLimits(params_.roi_z_min, params_.roi_z_max);
        pass_dist_.setFilterFieldName("x");
        
        voxel_near_.setLeafSize(params_.voxel_near_x, params_.voxel_near_y, params_.voxel_near_z);
        voxel_far_.setLeafSize(params_.voxel_far_x, params_.voxel_far_y, params_.voxel_far_z);

        seg_.setOptimizeCoefficients(true);
        seg_.setModelType(pcl::SACMODEL_PLANE);
        seg_.setMethodType(pcl::SAC_RANSAC);
        seg_.setMaxIterations(params_.ransac_max_iterations);
        seg_.setDistanceThreshold(params_.ransac_distance_threshold);
        extract_.setNegative(true);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensor/lidar/pointcloud", 10,
            [this](sensor_msgs::msg::PointCloud2::UniquePtr msg){this->cloud_callback(std::move(msg));}
        );
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/filtered_points", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar/cone_markers", 10);
        
        RCLCPP_INFO(this->get_logger(), "Pipeline Initialization Complete! Engine Ready.");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_; // 内存池
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacles_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_near_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_far_;

    pcl::PassThrough<pcl::PointXYZI> pass_x_, pass_y_, pass_z_, pass_dist_; // 算子池
    pcl::VoxelGrid<pcl::PointXYZI> voxel_near_, voxel_far_;
    pcl::SACSegmentation<pcl::PointXYZI> seg_;
    pcl::PointIndices::Ptr inliers_;
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::ExtractIndices<pcl::PointXYZI> extract_;
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_;

    struct Track { // 目标追踪状态机
        int id;
        float x, y, z;
        float sx, sy, sz; // 长宽高
        float vx, vy;
        int age;
        int time_since_update;
    };
    struct Detection {
        float x, y, z;
        float sx, sy, sz;
    };
    struct Params {
        float roi_x_min;
        float roi_x_max;
        float roi_y_min;
        float roi_y_max;
        float roi_z_min;
        float roi_z_max;

        float distance_near_min;
        float distance_near_max;
        float distance_far_min;
        float distance_far_max;

        float voxel_near_x;
        float voxel_near_y;
        float voxel_near_z;
        float voxel_far_x;
        float voxel_far_y;
        float voxel_far_z;

        int ransac_max_iterations;
        float ransac_distance_threshold;

        float cluster_base_radius;
        float cluster_radius_alpha;
        int cluster_min_neighbors;
        int cluster_min_size;
        int cluster_max_size;

        float detection_max_sx;
        float detection_max_sy;
        float detection_min_sz;
        float detection_max_sz;

        float timing_fallback_dt;
        float timing_max_dt;

        float tracking_match_thresh;
        float tracking_alpha;
        int tracking_max_missed;

        float marker_box_scale_min;
        float marker_lifetime;
        int marker_text_id_offset;
        float marker_text_offset_z;
        float marker_text_scale;
    };
    std::vector<Track> active_tracks_;
    int next_track_id_ = 0;
    Params params_;
    rclcpp::Time last_stamp_;
    bool has_last_stamp_ = false;

    void load_params() {
        params_.roi_x_min = this->declare_parameter<float>("roi.x_min", 0.5f);
        params_.roi_x_max = this->declare_parameter<float>("roi.x_max", 30.0f);
        params_.roi_y_min = this->declare_parameter<float>("roi.y_min", -15.0f);
        params_.roi_y_max = this->declare_parameter<float>("roi.y_max", 15.0f);
        params_.roi_z_min = this->declare_parameter<float>("roi.z_min", -2.0f);
        params_.roi_z_max = this->declare_parameter<float>("roi.z_max", 0.5f);

        params_.distance_near_min = this->declare_parameter<float>("distance.near_min", 1.5f);
        params_.distance_near_max = this->declare_parameter<float>("distance.near_max", 15.0f);
        params_.distance_far_min = this->declare_parameter<float>("distance.far_min", 15.0f);
        params_.distance_far_max = this->declare_parameter<float>("distance.far_max", 30.0f);

        params_.voxel_near_x = this->declare_parameter<float>("voxel.near_x", 0.1f);
        params_.voxel_near_y = this->declare_parameter<float>("voxel.near_y", 0.1f);
        params_.voxel_near_z = this->declare_parameter<float>("voxel.near_z", 0.05f);
        params_.voxel_far_x = this->declare_parameter<float>("voxel.far_x", 0.05f);
        params_.voxel_far_y = this->declare_parameter<float>("voxel.far_y", 0.05f);
        params_.voxel_far_z = this->declare_parameter<float>("voxel.far_z", 0.025f);

        params_.ransac_max_iterations = this->declare_parameter<int>("ground.ransac_max_iterations", 50);
        params_.ransac_distance_threshold = this->declare_parameter<float>("ground.ransac_distance_threshold", 0.18f);

        params_.cluster_base_radius = this->declare_parameter<float>("cluster.base_radius", 0.3f);
        params_.cluster_radius_alpha = this->declare_parameter<float>("cluster.radius_alpha", 0.015f);
        params_.cluster_min_neighbors = this->declare_parameter<int>("cluster.min_neighbors", 3);
        params_.cluster_min_size = this->declare_parameter<int>("cluster.min_size", 5);
        params_.cluster_max_size = this->declare_parameter<int>("cluster.max_size", 500);

        params_.detection_max_sx = this->declare_parameter<float>("detection.max_sx", 0.8f);
        params_.detection_max_sy = this->declare_parameter<float>("detection.max_sy", 0.8f);
        params_.detection_min_sz = this->declare_parameter<float>("detection.min_sz", 0.1f);
        params_.detection_max_sz = this->declare_parameter<float>("detection.max_sz", 1.2f);

        params_.timing_fallback_dt = this->declare_parameter<float>("timing.fallback_dt", 0.1f);
        params_.timing_max_dt = this->declare_parameter<float>("timing.max_dt", 0.5f);

        params_.tracking_match_thresh = this->declare_parameter<float>("tracking.match_thresh", 1.0f);
        params_.tracking_alpha = this->declare_parameter<float>("tracking.alpha", 0.8f);
        params_.tracking_max_missed = this->declare_parameter<int>("tracking.max_missed", 3);

        params_.marker_box_scale_min = this->declare_parameter<float>("marker.box_scale_min", 0.2f);
        params_.marker_lifetime = this->declare_parameter<float>("marker.lifetime", 0.15f);
        params_.marker_text_id_offset = this->declare_parameter<int>("marker.text_id_offset", 10000);
        params_.marker_text_offset_z = this->declare_parameter<float>("marker.text_offset_z", 0.8f);
        params_.marker_text_scale = this->declare_parameter<float>("marker.text_scale", 0.4f);
    }

    void cloud_callback(std::unique_ptr<sensor_msgs::msg::PointCloud2> msg) {
        auto start_time = std::chrono::steady_clock::now();

        pcl_cloud_->clear(); cloud_roi_->clear(); cloud_filtered_->clear(); // 清空上一帧
        cloud_obstacles_->clear(); cloud_near_->clear(); cloud_far_->clear();

        pcl::fromROSMsg(*msg, *pcl_cloud_); // 类型转换

        preprocess_cloud();

        remove_ground();

        float dt = params_.timing_fallback_dt;
        const rclcpp::Time current_stamp(msg->header.stamp);
        if (has_last_stamp_) {
            dt = (current_stamp - last_stamp_).seconds();
            if (dt <= 0.0f || dt > params_.timing_max_dt) {
                dt = params_.timing_fallback_dt;
            }
        }
        last_stamp_ = current_stamp;
        has_last_stamp_ = true;

        cluster_and_track(msg->header, dt);

        auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud_obstacles_, *output_msg);
        output_msg->header = msg->header;
        pub_->publish(std::move(output_msg));

        auto end_time = std::chrono::steady_clock::now();
        double cost_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        RCLCPP_INFO(this->get_logger(), "Pipeline Cost: %.2f ms | Tracking %zu cones", cost_ms, active_tracks_.size());
    }

    void preprocess_cloud() { // 预处理
        pass_x_.setInputCloud(pcl_cloud_); pass_x_.filter(*cloud_roi_); // ROI
        pass_y_.setInputCloud(cloud_roi_); pass_y_.filter(*cloud_roi_);
        pass_z_.setInputCloud(cloud_roi_); pass_z_.filter(*cloud_roi_);

        pass_dist_.setInputCloud(cloud_roi_); // 分层
        pass_dist_.setFilterLimits(params_.distance_near_min, params_.distance_near_max);
        pass_dist_.filter(*cloud_near_);
        pass_dist_.setFilterLimits(params_.distance_far_min, params_.distance_far_max);
        pass_dist_.filter(*cloud_far_);

        pcl::PointCloud<pcl::PointXYZI> temp_near, temp_far; // 降采样
        voxel_near_.setInputCloud(cloud_near_); voxel_near_.filter(temp_near);
        voxel_far_.setInputCloud(cloud_far_);   voxel_far_.filter(temp_far);

        *cloud_filtered_ = temp_near + temp_far;
    }

    void remove_ground() { // RANSAC
        if (cloud_filtered_->empty()) {
            cloud_obstacles_->clear();
            return;
        }

        seg_.setInputCloud(cloud_filtered_);
        seg_.segment(*inliers_, *coefficients_);

        if (!inliers_->indices.empty()) {
            extract_.setInputCloud(cloud_filtered_);
            extract_.setIndices(inliers_);
            extract_.filter(*cloud_obstacles_);
        } else {
            RCLCPP_WARN(this->get_logger(), "Ground lost! Fallback to filtered.");
            *cloud_obstacles_ = *cloud_filtered_;
        }
        return;
    }

    void cluster_and_track(const std_msgs::msg::Header& header, float dt) { // KdTree + 欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        build_clusters(cluster_indices);

        std::vector<Detection> detections;
        build_detections(cluster_indices, detections);

        update_tracks(detections, dt);
        publish_markers(header);
    }

    void build_clusters(std::vector<pcl::PointIndices>& cluster_indices) {
        cluster_indices.clear();
        if (cloud_obstacles_->empty()) {
            return;
        }

        kdtree_->setInputCloud(cloud_obstacles_);
        const int num_points = static_cast<int>(cloud_obstacles_->size());
        std::vector<bool> processed(num_points, false);

        const float BASE_RADIUS = params_.cluster_base_radius;
        const float ALPHA = params_.cluster_radius_alpha;
        const int MIN_NEIGHBORS = params_.cluster_min_neighbors;
        const int MIN_CLUSTER_SIZE = params_.cluster_min_size;
        const int MAX_CLUSTER_SIZE = params_.cluster_max_size;

        std::vector<int> neighbors;
        std::vector<float> sq_distances;
        neighbors.reserve(128);
        sq_distances.reserve(128);

        for (int i = 0; i < num_points; ++i) { // BFS
            if (processed[i]) continue;
            std::vector<int> current_cluster;
            current_cluster.reserve(64);
            std::queue<int> seed_queue;

            seed_queue.push(i);
            processed[i] = true;

            while (!seed_queue.empty()) {
                int curr_idx = seed_queue.front();
                seed_queue.pop();
                current_cluster.push_back(curr_idx);

                const auto& pt = cloud_obstacles_->points[curr_idx];
                float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
                float dynamic_radius = BASE_RADIUS + ALPHA * dist;

                neighbors.clear();
                sq_distances.clear();
                kdtree_->radiusSearch(pt, dynamic_radius, neighbors, sq_distances);

                if (neighbors.size() < static_cast<size_t>(MIN_NEIGHBORS)) continue;

                for (int n_idx : neighbors) {
                    if (!processed[n_idx]) {
                        processed[n_idx] = true;
                        seed_queue.push(n_idx);
                    }
                }
            }
            if (current_cluster.size() >= static_cast<size_t>(MIN_CLUSTER_SIZE) &&
                current_cluster.size() <= static_cast<size_t>(MAX_CLUSTER_SIZE)) {
                pcl::PointIndices pi;
                pi.indices = std::move(current_cluster);
                cluster_indices.push_back(std::move(pi));
            }
        }
    }

    void build_detections(const std::vector<pcl::PointIndices>& cluster_indices,
                          std::vector<Detection>& detections) {
        detections.clear();
        detections.reserve(cluster_indices.size());

        for (const auto& cluster : cluster_indices) { // 提取当前帧观测值
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*cloud_obstacles_, cluster.indices, min_pt, max_pt);

            float sx = max_pt[0] - min_pt[0];
            float sy = max_pt[1] - min_pt[1];
            float sz = max_pt[2] - min_pt[2];

            if (sx > params_.detection_max_sx || sy > params_.detection_max_sy ||
                sz > params_.detection_max_sz || sz < params_.detection_min_sz) {
                continue;
            }

            detections.push_back({
                (min_pt[0] + max_pt[0]) * 0.5f,
                (min_pt[1] + max_pt[1]) * 0.5f,
                (min_pt[2] + max_pt[2]) * 0.5f,
                sx, sy, sz
            });
        }
    }

    void update_tracks(const std::vector<Detection>& detections, float dt) {
        std::vector<float> prev_x(active_tracks_.size());
        std::vector<float> prev_y(active_tracks_.size());

        for (size_t i = 0; i < active_tracks_.size(); ++i) {
            auto& track = active_tracks_[i];
            prev_x[i] = track.x;
            prev_y[i] = track.y;
            track.x += track.vx * dt;
            track.y += track.vy * dt;
            track.time_since_update++;
        }

        const float MATCH_THRESH_SQ = params_.tracking_match_thresh * params_.tracking_match_thresh;
        std::vector<bool> matched_detections(detections.size(), false);
        const float inv_dt = (dt > 1e-3f) ? (1.0f / dt) : 0.0f;

        for (size_t t_idx = 0; t_idx < active_tracks_.size(); ++t_idx) { // 目标追踪
            auto& track = active_tracks_[t_idx];
            float min_dist_sq = MATCH_THRESH_SQ;
            int best_det_idx = -1;

            for (size_t i = 0; i < detections.size(); ++i) {
                if (matched_detections[i]) continue;
                float dx = track.x - detections[i].x;
                float dy = track.y - detections[i].y;
                float dist_sq = dx * dx + dy * dy;

                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    best_det_idx = static_cast<int>(i);
                }
            }

            if (best_det_idx != -1) {
                const float ALPHA_TRACK = params_.tracking_alpha;
                float mx = detections[best_det_idx].x;
                float my = detections[best_det_idx].y;

                track.x = track.x + ALPHA_TRACK * (mx - track.x);
                track.y = track.y + ALPHA_TRACK * (my - track.y);
                track.vx = (track.x - prev_x[t_idx]) * inv_dt;
                track.vy = (track.y - prev_y[t_idx]) * inv_dt;
                track.z = detections[best_det_idx].z;
                track.sx = detections[best_det_idx].sx;
                track.sy = detections[best_det_idx].sy;
                track.sz = detections[best_det_idx].sz;

                track.time_since_update = 0;
                track.age++;
                matched_detections[best_det_idx] = true;
            }
        }

        for (size_t i = 0; i < detections.size(); ++i) {
            if (!matched_detections[i]) {
                Track t;
                t.id = next_track_id_++;
                t.x = detections[i].x; t.y = detections[i].y; t.z = detections[i].z;
                t.sx = detections[i].sx; t.sy = detections[i].sy; t.sz = detections[i].sz;
                t.vx = 0.0f; t.vy = 0.0f;
                t.age = 1; t.time_since_update = 0;
                active_tracks_.push_back(t);
            }
        }

        active_tracks_.erase(
            std::remove_if(active_tracks_.begin(), active_tracks_.end(),
                           [this](const Track& t) { return t.time_since_update > params_.tracking_max_missed; }),
            active_tracks_.end()
        );
    }

    void publish_markers(const std_msgs::msg::Header& header) {
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(delete_all);

        for (const auto& track : active_tracks_) {
            if (track.time_since_update > 0) continue; // 只画当前帧真实存在的

            // 框体
            visualization_msgs::msg::Marker box;
            box.header = header;
            box.ns = "cone_boxes";
            box.id = track.id; // 绑定ID
            box.type = visualization_msgs::msg::Marker::CUBE;
            box.action = visualization_msgs::msg::Marker::ADD;
            box.pose.position.x = track.x;
            box.pose.position.y = track.y;
            box.pose.position.z = track.z;
            box.scale.x = std::max(params_.marker_box_scale_min, track.sx);
            box.scale.y = std::max(params_.marker_box_scale_min, track.sy);
            box.scale.z = std::max(params_.marker_box_scale_min, track.sz);
            box.color.r = 0.0f; box.color.g = 1.0f; box.color.b = 0.0f; box.color.a = 0.5f;
            box.lifetime = rclcpp::Duration::from_seconds(params_.marker_lifetime);
            marker_array.markers.push_back(box);

            // ID 数字
            visualization_msgs::msg::Marker text;
            text.header = header;
            text.ns = "cone_ids";
            text.id = track.id + params_.marker_text_id_offset; // 避免 ID 冲突
            text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text.action = visualization_msgs::msg::Marker::ADD;
            text.pose.position.x = track.x;
            text.pose.position.y = track.y;
            text.pose.position.z = track.z + params_.marker_text_offset_z;
            text.scale.z = params_.marker_text_scale; // 字体大小
            text.color.r = 1.0f; text.color.g = 1.0f; text.color.b = 1.0f; text.color.a = 1.0f;
            text.text = std::to_string(track.id);
            text.lifetime = rclcpp::Duration::from_seconds(params_.marker_lifetime);
            marker_array.markers.push_back(text);
        }

        marker_pub_->publish(marker_array);
    }
};

} // namespace as_training

RCLCPP_COMPONENTS_REGISTER_NODE(as_training::LidarPerceptionNode)