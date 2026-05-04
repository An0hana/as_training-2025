#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

#include <Eigen/Dense>
#include <patchwork/patchworkpp.h>

namespace as_training {

class HungarianAlgorithm { // 匈牙利算法实现二分图最大权匹配
public:
  static void Solve(const std::vector<std::vector<float>> &DistMatrix,
                    std::vector<int> &Assignment) {
    int nRows = DistMatrix.size();
    int nCols = DistMatrix[0].size();

    int dim = std::max(nRows, nCols); // Pad矩阵设为方阵
    std::vector<std::vector<float>> cost(dim, std::vector<float>(dim, 0.0f));
    float max_cost = 0;

    for (int r = 0; r < nRows; r++) {
      for (int c = 0; c < nCols; c++) {
        cost[r][c] = DistMatrix[r][c];
        if (cost[r][c] > max_cost)
          max_cost = cost[r][c];
      }
    }

    for (int r = 0; r < dim; r++) { // 非方阵填充最大值
      for (int c = 0; c < dim; c++) {
        if (r >= nRows || c >= nCols)
          cost[r][c] = max_cost;
      }
    }

    std::vector<float> lx(dim, 0), ly(dim, 0);
    std::vector<int> xy(dim, -1), yx(dim, -1);
    std::vector<bool> S(dim), T(dim);
    std::vector<float> slack(dim);
    std::vector<int> slackx(dim);

    for (int i = 0; i < dim; i++) {
      lx[i] = cost[i][0];
      for (int j = 1; j < dim; j++) {
        if (cost[i][j] < lx[i])
          lx[i] = cost[i][j];
      }
    }

    for (int root = 0; root < dim; root++) {
      std::fill(slack.begin(), slack.end(), std::numeric_limits<float>::max());
      std::fill(slackx.begin(), slackx.end(), 0);
      std::fill(S.begin(), S.end(), false);
      std::fill(T.begin(), T.end(), false);

      int x = root, y = -1, py = -1;
      S[x] = true;

      for (int i = 0; i < dim; i++) {
        if (cost[x][i] - lx[x] - ly[i] < slack[i]) {
          slack[i] = cost[x][i] - lx[x] - ly[i];
          slackx[i] = x;
        }
      }

      while (true) {
        float delta = std::numeric_limits<float>::max();
        for (int i = 0; i < dim; i++) {
          if (!T[i] && slack[i] < delta) {
            delta = slack[i];
            y = i;
          }
        }

        for (int i = 0; i < dim; i++) {
          if (S[i])
            lx[i] += delta;
          if (T[i])
            ly[i] -= delta;
          else
            slack[i] -= delta;
        }

        T[y] = true;
        py = slackx[y];
        if (yx[y] == -1)
          break;

        x = yx[y];
        S[x] = true;
        for (int i = 0; i < dim; i++) {
          if (!T[i] && cost[x][i] - lx[x] - ly[i] < slack[i]) {
            slack[i] = cost[x][i] - lx[x] - ly[i];
            slackx[i] = x;
          }
        }
      }

      while (py != -1) {
        int ty = xy[py];
        yx[y] = py;
        xy[py] = y;
        y = ty;
        if (y != -1)
          py = slackx[y];
        else
          py = -1;
      }
    }

    Assignment.clear();
    for (int r = 0; r < nRows; r++) {
      if (xy[r] < nCols)
        Assignment.push_back(xy[r]);
      else
        Assignment.push_back(-1);
    }
  }
};

class LidarPerceptionNode : public rclcpp::Node {
public:
  explicit LidarPerceptionNode(const rclcpp::NodeOptions &options)
      : Node("lidar_perception_node",
             rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
    RCLCPP_INFO(this->get_logger(), "Constructing lidar_perception_node...");

    pcl_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_roi_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_filtered_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cloud_obstacles_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    load_params();

    patchwork::Params pw_params;
    pw_params.sensor_height = 1.2;
    patchwork_ = std::make_unique<patchwork::PatchWorkpp>(pw_params);

    Eigen::Vector4f roi_min(params_.roi_x_min, params_.roi_y_min,
                            params_.roi_z_min, 1.0f);
    Eigen::Vector4f roi_max(params_.roi_x_max, params_.roi_y_max,
                            params_.roi_z_max, 1.0f);
    crop_box_.setMin(roi_min);
    crop_box_.setMax(roi_max);
    crop_box_.setNegative(false);

    Q_ = Eigen::Matrix4f::Zero(); // 初始化过程噪声和观测噪声
    Q_(0, 0) = 0.1f;
    Q_(1, 1) = 0.1f; // 位置不确定性
    Q_(2, 2) = 10.0f;
    Q_(3, 3) = 10.0f; // 速度极度不确定

    R_ = Eigen::Matrix2f::Zero();
    R_(0, 0) = 0.05f;
    R_(1, 1) = 0.05f; // 观测精度

    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/sensor/lidar/pointcloud", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
          this->cloud_callback(std::move(msg));
        });
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/lidar/filtered_points", rclcpp::SensorDataQoS());
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/lidar/cone_markers", 10);

    RCLCPP_INFO(this->get_logger(),
                "Done with constructing lidar_perception_node.");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_obstacles_;

  pcl::CropBox<pcl::PointXYZI> crop_box_;

  std::unique_ptr<patchwork::PatchWorkpp> patchwork_;
  Eigen::MatrixXf cloud_eigen_;

  std::vector<int> grid_head_;   // BEV 栅格化内存池
  std::vector<int> point_next_;
  std::vector<uint8_t> grid_label_;
  std::vector<int> bfs_queue_;

  struct Track { // 卡尔曼状态机
    int id;
    Eigen::Vector4f x;
    Eigen::Matrix4f P;
    float z, sx, sy, sz;
    int age;
    int time_since_update;

    void predict(float dt, const Eigen::Matrix4f &Q) {
      Eigen::Matrix4f F = Eigen::Matrix4f::Identity();
      F(0, 2) = dt;
      F(1, 3) = dt;
      x = F * x;
      P = F * P * F.transpose() + Q;
    }

    void update(const Eigen::Vector2f &z_meas, const Eigen::Matrix2f &R) {
      Eigen::Matrix<float, 2, 4> H;
      H << 1, 0, 0, 0, 0, 1, 0, 0;
      Eigen::Matrix2f S = H * P * H.transpose() + R;
      Eigen::Matrix<float, 4, 2> K = P * H.transpose() * S.inverse();
      Eigen::Vector2f y = z_meas - H * x;
      x = x + K * y;
      P = (Eigen::Matrix4f::Identity() - K * H) * P;
    }
  };

  struct Detection {
    float x, y, z;
    float sx, sy, sz;
  };

  struct Params {
    float roi_x_min, roi_x_max;
    float roi_y_min, roi_y_max;
    float roi_z_min, roi_z_max;

    float distance_range_min, distance_range_max;
    int distance_layer_count;

    float voxel_near_x, voxel_near_y, voxel_near_z;
    float voxel_far_x, voxel_far_y, voxel_far_z;

    int ransac_max_iterations;
    float ransac_distance_threshold;

    float bev_grid_res;

    int cluster_min_size;
    int cluster_max_size;

    float detection_max_sx, detection_max_sy;
    float detection_min_sz, detection_max_sz;

    float timing_fallback_dt;
    float timing_max_dt;

    float tracking_match_thresh;
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

  Eigen::Matrix4f Q_; // 系统噪声
  Eigen::Matrix2f R_; // 测量噪声

  void load_params() {
    params_.roi_x_min = this->declare_parameter<float>("roi.x_min", 0.5f);
    params_.roi_x_max = this->declare_parameter<float>("roi.x_max", 30.0f);
    params_.roi_y_min = this->declare_parameter<float>("roi.y_min", -15.0f);
    params_.roi_y_max = this->declare_parameter<float>("roi.y_max", 15.0f);
    params_.roi_z_min = this->declare_parameter<float>("roi.z_min", -2.0f);
    params_.roi_z_max = this->declare_parameter<float>("roi.z_max", 0.5f);

    params_.distance_range_min =
        this->declare_parameter<float>("distance.range_min", 1.5f);
    params_.distance_range_max =
        this->declare_parameter<float>("distance.range_max", 30.0f);
    params_.distance_layer_count =
        this->declare_parameter<int>("distance.layer_count", 3);

    params_.voxel_near_x = this->declare_parameter<float>("voxel.near_x", 0.1f);
    params_.voxel_near_y = this->declare_parameter<float>("voxel.near_y", 0.1f);
    params_.voxel_near_z =
        this->declare_parameter<float>("voxel.near_z", 0.05f);
    params_.voxel_far_x = this->declare_parameter<float>("voxel.far_x", 0.05f);
    params_.voxel_far_y = this->declare_parameter<float>("voxel.far_y", 0.05f);
    params_.voxel_far_z = this->declare_parameter<float>("voxel.far_z", 0.025f);

    params_.ransac_max_iterations =
        this->declare_parameter<int>("ground.ransac_max_iterations", 50);
    params_.ransac_distance_threshold = this->declare_parameter<float>(
        "ground.ransac_distance_threshold", 0.18f);

    params_.bev_grid_res =
        this->declare_parameter<float>("cluster.bev_grid_res", 0.15f);

    params_.cluster_min_size =
        this->declare_parameter<int>("cluster.min_size", 5);
    params_.cluster_max_size =
        this->declare_parameter<int>("cluster.max_size", 500);

    params_.detection_max_sx =
        this->declare_parameter<float>("detection.max_sx", 0.8f);
    params_.detection_max_sy =
        this->declare_parameter<float>("detection.max_sy", 0.8f);
    params_.detection_min_sz =
        this->declare_parameter<float>("detection.min_sz", 0.1f);
    params_.detection_max_sz =
        this->declare_parameter<float>("detection.max_sz", 1.2f);

    params_.timing_fallback_dt =
        this->declare_parameter<float>("timing.fallback_dt", 0.1f);
    params_.timing_max_dt =
        this->declare_parameter<float>("timing.max_dt", 0.5f);

    params_.tracking_match_thresh =
        this->declare_parameter<float>("tracking.match_thresh", 1.0f);
    params_.tracking_max_missed =
        this->declare_parameter<int>("tracking.max_missed", 3);

    params_.marker_box_scale_min =
        this->declare_parameter<float>("marker.box_scale_min", 0.2f);
    params_.marker_lifetime =
        this->declare_parameter<float>("marker.lifetime", 0.30f);
    params_.marker_text_id_offset =
        this->declare_parameter<int>("marker.text_id_offset", 10000);
    params_.marker_text_offset_z =
        this->declare_parameter<float>("marker.text_offset_z", 0.8f);
    params_.marker_text_scale =
        this->declare_parameter<float>("marker.text_scale", 0.4f);

    bfs_queue_.reserve(2048);
  }

  void cloud_callback(std::unique_ptr<sensor_msgs::msg::PointCloud2> msg) {
    auto start_time = std::chrono::steady_clock::now();

    cloud_roi_->clear();
    cloud_filtered_->clear();
    cloud_obstacles_->clear();

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    bool has_intensity = false;
    for (const auto &field : msg->fields) {
      if (field.name == "intensity") {
        has_intensity = true;
        break;
      }
    }

    const float min_r_sq =
        params_.distance_range_min * params_.distance_range_min;
    const float max_r_sq =
        params_.distance_range_max * params_.distance_range_max;

    cloud_roi_->points.reserve(msg->width * msg->height / 2);

    if (has_intensity) {
      sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_i(*msg, "intensity");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
        float x = *iter_x, y = *iter_y, z = *iter_z;
        if (x < params_.roi_x_min || x > params_.roi_x_max ||
            y < params_.roi_y_min || y > params_.roi_y_max ||
            z < params_.roi_z_min || z > params_.roi_z_max)
          continue;
        float r_sq = x * x + y * y;
        if (r_sq < min_r_sq || r_sq > max_r_sq)
          continue;
        pcl::PointXYZI pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = static_cast<float>(*iter_i);
        cloud_roi_->points.push_back(pt);
      }
    } else {
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x, y = *iter_y, z = *iter_z;
        if (x < params_.roi_x_min || x > params_.roi_x_max ||
            y < params_.roi_y_min || y > params_.roi_y_max ||
            z < params_.roi_z_min || z > params_.roi_z_max)
          continue;
        float r_sq = x * x + y * y;
        if (r_sq < min_r_sq || r_sq > max_r_sq)
          continue;
        pcl::PointXYZI pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.intensity = 0.0f;
        cloud_roi_->points.push_back(pt);
      }
    }

    auto t1 = std::chrono::steady_clock::now();
    auto t2 = t1; // cut_roi logic fused

    downsampling();
    auto t3 = std::chrono::steady_clock::now();

    remove_ground();
    auto t4 = std::chrono::steady_clock::now();

    float dt = params_.timing_fallback_dt;
    const rclcpp::Time current_stamp(msg->header.stamp);
    if (has_last_stamp_) {
      dt = (current_stamp - last_stamp_).seconds();
      if (dt <= 0.0f || dt > params_.timing_max_dt)
        dt = params_.timing_fallback_dt;
    }
    last_stamp_ = current_stamp;
    has_last_stamp_ = true;

    cluster_and_track(msg->header, dt);
    auto t5 = std::chrono::steady_clock::now();

    auto output_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud_obstacles_, *output_msg);
    output_msg->header = msg->header;
    pub_->publish(std::move(output_msg));

    auto end_time = std::chrono::steady_clock::now();

    double cost_pcl =
        std::chrono::duration<double, std::milli>(t1 - start_time).count();
    double cost_roi =
        std::chrono::duration<double, std::milli>(t2 - t1).count();
    double cost_downsample =
        std::chrono::duration<double, std::milli>(t3 - t2).count();
    double cost_ground =
        std::chrono::duration<double, std::milli>(t4 - t3).count();
    double cost_track =
        std::chrono::duration<double, std::milli>(t5 - t4).count();
    double cost_pub =
        std::chrono::duration<double, std::milli>(end_time - t5).count();
    double cost_total =
        std::chrono::duration<double, std::milli>(end_time - start_time)
            .count();

    RCLCPP_INFO(this->get_logger(),
                "Time [Total: %.2fms] PCL: %.2f | ROI: %.2f | Grnd: %.2f | DS: "
                "%.2f | Track: %.2f | Pub: %.2f | Cones: %zu",
                cost_total, cost_pcl, cost_roi, cost_ground, cost_downsample,
                cost_track, cost_pub, active_tracks_.size());
  }

  void remove_ground() {
    if (cloud_roi_->empty()) {
      cloud_obstacles_->clear();
      return;
    }
    cloud_obstacles_->clear();
    const size_t num_points = cloud_roi_->size();
    if (static_cast<size_t>(cloud_eigen_.rows()) < num_points)
      cloud_eigen_.resize(num_points * 1.2, 3);
    for (size_t i = 0; i < num_points; ++i) {
      cloud_eigen_(i, 0) = cloud_roi_->points[i].x;
      cloud_eigen_(i, 1) = cloud_roi_->points[i].y;
      cloud_eigen_(i, 2) = cloud_roi_->points[i].z;
    }
    patchwork_->estimateGround(cloud_eigen_.block(0, 0, num_points, 3));
    Eigen::VectorXi nonground_idx = patchwork_->getNongroundIndices();
    cloud_obstacles_->reserve(nonground_idx.size());
    for (int i = 0; i < nonground_idx.size(); ++i) {
      cloud_obstacles_->push_back(cloud_roi_->points[nonground_idx(i)]);
    }
    if (cloud_obstacles_->empty())
      *cloud_obstacles_ = *cloud_roi_;
  }

  void downsampling() {
    cloud_filtered_->clear();
    if (cloud_roi_->empty())
      return;

    const int HASH_SIZE = 500009;
    static std::vector<int> hash_table(HASH_SIZE, -1);
    static std::vector<int> active_hashes;
    active_hashes.reserve(cloud_roi_->size());

    const float min_r = params_.distance_range_min;
    const float max_r = params_.distance_range_max;
    const int layer_count = std::max(2, params_.distance_layer_count);
    if (max_r <= min_r)
      return;

    const float step = (max_r - min_r) / static_cast<float>(layer_count);
    cloud_filtered_->points.reserve(cloud_roi_->size() / 2);

    const float near_x = std::min(params_.voxel_near_x, params_.voxel_far_x);
    const float far_x = std::max(params_.voxel_near_x, params_.voxel_far_x);
    const float near_y = std::min(params_.voxel_near_y, params_.voxel_far_y);
    const float far_y = std::max(params_.voxel_near_y, params_.voxel_far_y);
    const float near_z = std::min(params_.voxel_near_z, params_.voxel_far_z);
    const float far_z = std::max(params_.voxel_near_z, params_.voxel_far_z);

    for (const auto &pt : cloud_roi_->points) {
      float r = std::hypot(pt.x, pt.y);

      int layer_idx = static_cast<int>((r - min_r) / step); // 分层设置动态体素
      layer_idx = std::clamp(layer_idx, 0, layer_count - 1);
      float layer_center =
          min_r + step * (static_cast<float>(layer_idx) + 0.5f);
      float t =
          std::clamp((layer_center - min_r) / (max_r - min_r), 0.0f, 1.0f);

      float leaf_x = near_x + t * (far_x - near_x);
      float leaf_y = near_y + t * (far_y - near_y);
      float leaf_z = near_z + t * (far_z - near_z);

      int hx = static_cast<int>(std::floor(pt.x / leaf_x)); // 整数网格坐标
      int hy = static_cast<int>(std::floor(pt.y / leaf_y));
      int hz = static_cast<int>(std::floor(pt.z / leaf_z));

      int hash_val = (((hx * 73856093) ^ (hy * 19349663) ^ (hz * 83492791)) % HASH_SIZE); // 空间质数哈希
      if (hash_val < 0)
        hash_val += HASH_SIZE;

      if (hash_table[hash_val] == -1) { // 存入第一个遇到该格子的点
        hash_table[hash_val] = 1;
        active_hashes.push_back(hash_val);
        cloud_filtered_->points.push_back(pt);
      }
    }

    for (int hash_idx : active_hashes) { // 清理
      hash_table[hash_idx] = -1;
    }
    active_hashes.clear();

    cloud_roi_.swap(cloud_filtered_);
  }

  void build_clusters(std::vector<pcl::PointIndices> &cluster_indices) {
    cluster_indices.clear();
    if (cloud_obstacles_->empty())
      return;
    const float res = params_.bev_grid_res;
    const float inv_res = 1.0f / res;
    const int W = std::ceil((params_.roi_y_max - params_.roi_y_min) * inv_res);
    const int H = std::ceil((params_.roi_x_max - params_.roi_x_min) * inv_res);
    if (W <= 0 || H <= 0)
      return;
    const int num_cells = W * H;
    const int num_points = cloud_obstacles_->size();
    grid_head_.assign(num_cells, -1);
    grid_label_.assign(num_cells, 0);
    point_next_.assign(num_points, -1);
    for (int i = 0; i < num_points; ++i) {
      const auto &pt = cloud_obstacles_->points[i];
      int u = std::floor((pt.y - params_.roi_y_min) * inv_res);
      int v = std::floor((pt.x - params_.roi_x_min) * inv_res);
      if (u < 0 || u >= W || v < 0 || v >= H)
        continue;
      int idx = v * W + u;
      point_next_[i] = grid_head_[idx];
      grid_head_[idx] = i;
      grid_label_[idx] = 1;
    }
    const std::array<int, 8> dx = {1, -1, 0, 0, 1, 1, -1, -1};
    const std::array<int, 8> dy = {0, 0, 1, -1, 1, -1, 1, -1};
    std::vector<int> current_cluster_pts;
    current_cluster_pts.reserve(1024);
    for (int i = 0; i < num_cells; ++i) {
      if (grid_label_[i] == 1) {
        bfs_queue_.clear();
        bfs_queue_.push_back(i);
        grid_label_[i] = 2;
        current_cluster_pts.clear();
        int q_head = 0;
        while (q_head < static_cast<int>(bfs_queue_.size())) {
          int curr = bfs_queue_[q_head++];
          int pt_idx = grid_head_[curr];
          while (pt_idx != -1) {
            current_cluster_pts.push_back(pt_idx);
            pt_idx = point_next_[pt_idx];
          }
          int cu = curr % W, cv = curr / W;
          for (int d = 0; d < 8; ++d) {
            int nu = cu + dx[d], nv = cv + dy[d];
            if (nu >= 0 && nu < W && nv >= 0 && nv < H) {
              int n_idx = nv * W + nu;
              if (grid_label_[n_idx] == 1) {
                grid_label_[n_idx] = 2;
                bfs_queue_.push_back(n_idx);
              }
            }
          }
        }
        if (current_cluster_pts.size() >=
                static_cast<size_t>(params_.cluster_min_size) &&
            current_cluster_pts.size() <=
                static_cast<size_t>(params_.cluster_max_size)) {
          pcl::PointIndices pi;
          pi.indices = current_cluster_pts;
          cluster_indices.push_back(std::move(pi));
        }
      }
    }
  }

  void build_detections(const std::vector<pcl::PointIndices> &cluster_indices,
                        std::vector<Detection> &detections) {
    detections.clear();
    detections.reserve(cluster_indices.size());
    for (const auto &cluster : cluster_indices) {
      Eigen::Vector4f min_pt, max_pt;
      pcl::getMinMax3D(*cloud_obstacles_, cluster.indices, min_pt, max_pt);
      float sx = max_pt[0] - min_pt[0], sy = max_pt[1] - min_pt[1],
            sz = max_pt[2] - min_pt[2];
      if (sx > params_.detection_max_sx || sy > params_.detection_max_sy ||
          sz > params_.detection_max_sz || sz < params_.detection_min_sz)
        continue;
      detections.push_back({(min_pt[0] + max_pt[0]) * 0.5f,
                            (min_pt[1] + max_pt[1]) * 0.5f,
                            (min_pt[2] + max_pt[2]) * 0.5f, sx, sy, sz});
    }
  }


  // 卡尔曼滤波 + 匈牙利算法
  void cluster_and_track(const std_msgs::msg::Header &header, float dt) {
    std::vector<pcl::PointIndices> cluster_indices;
    build_clusters(cluster_indices);

    std::vector<Detection> detections;
    build_detections(cluster_indices, detections);

    for (auto &track : active_tracks_) { // 卡尔曼预测
      track.predict(dt, Q_);
      track.time_since_update++;
    }

    const float MATCH_THRESH_SQ = params_.tracking_match_thresh * params_.tracking_match_thresh; // 构建代价矩阵
    std::vector<std::vector<float>> cost_matrix(
        active_tracks_.size(), std::vector<float>(detections.size(), 0.0f));

    for (size_t t = 0; t < active_tracks_.size(); ++t) {
      for (size_t d = 0; d < detections.size(); ++d) {
        float dx = active_tracks_[t].x(0) - detections[d].x;
        float dy = active_tracks_[t].x(1) - detections[d].y;
        float dist_sq = dx * dx + dy * dy;

        cost_matrix[t][d] = (dist_sq < MATCH_THRESH_SQ) ? dist_sq : 9999.0f; // 处理远距离目标
      }
    }

    std::vector<int> assignments; // 匈牙利匹配求解全局最优
    if (!cost_matrix.empty() && !cost_matrix[0].empty()) {
      HungarianAlgorithm::Solve(cost_matrix, assignments);
    }

    std::vector<bool> matched_detections(detections.size(), false);

    for (size_t t = 0; t < assignments.size(); ++t) { // 卡尔曼更新
      int d_idx = assignments[t];
      if (d_idx != -1 && cost_matrix[t][d_idx] < MATCH_THRESH_SQ) {
        Eigen::Vector2f z_meas(detections[d_idx].x, detections[d_idx].y);
        active_tracks_[t].update(z_meas, R_);

        active_tracks_[t].z = detections[d_idx].z;
        active_tracks_[t].sx = detections[d_idx].sx;
        active_tracks_[t].sy = detections[d_idx].sy;
        active_tracks_[t].sz = detections[d_idx].sz;

        active_tracks_[t].time_since_update = 0;
        active_tracks_[t].age++;
        matched_detections[d_idx] = true;
      }
    }

    for (size_t i = 0; i < detections.size(); ++i) { // 创建新 Track
      if (!matched_detections[i]) {
        Track t;
        t.id = next_track_id_++;
        t.x << detections[i].x, detections[i].y, 0.0f,
            0.0f;

        t.P = Eigen::Matrix4f::Identity(); // 初始协方差，位置确定，速度不确定
        t.P(2, 2) = 50.0f;
        t.P(3, 3) = 50.0f;

        t.z = detections[i].z;
        t.sx = detections[i].sx;
        t.sy = detections[i].sy;
        t.sz = detections[i].sz;
        t.age = 1;
        t.time_since_update = 0;
        active_tracks_.push_back(t);
      }
    }

    active_tracks_.erase(std::remove_if(active_tracks_.begin(), // 处理死亡 Track 
                                        active_tracks_.end(),
                                        [this](const Track &t) {
                                          return t.time_since_update >
                                                 params_.tracking_max_missed;
                                        }),
                         active_tracks_.end());

    publish_markers(header);
  }

  void publish_markers(const std_msgs::msg::Header &header) {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    for (const auto &track : active_tracks_) {
      if (track.time_since_update > 0)
        continue;

      visualization_msgs::msg::Marker box;
      box.header = header;
      box.ns = "cone_boxes";
      box.id = track.id;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;
      box.pose.position.x = track.x(0); // 从状态向量取 x
      box.pose.position.y = track.x(1); // 从状态向量取 y
      box.pose.position.z = track.z;
      box.scale.x = std::max(params_.marker_box_scale_min, track.sx);
      box.scale.y = std::max(params_.marker_box_scale_min, track.sy);
      box.scale.z = std::max(params_.marker_box_scale_min, track.sz);
      box.color.r = 0.0f;
      box.color.g = 1.0f;
      box.color.b = 0.0f;
      box.color.a = 0.5f;
      box.lifetime = rclcpp::Duration::from_seconds(params_.marker_lifetime);
      marker_array.markers.push_back(box);

      visualization_msgs::msg::Marker text;
      text.header = header;
      text.ns = "cone_ids";
      text.id = track.id + params_.marker_text_id_offset;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = track.x(0);
      text.pose.position.y = track.x(1);
      text.pose.position.z = track.z + params_.marker_text_offset_z;
      text.scale.z = params_.marker_text_scale;
      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text = std::to_string(track.id);
      text.lifetime = rclcpp::Duration::from_seconds(params_.marker_lifetime);
      marker_array.markers.push_back(text);
    }

    marker_pub_->publish(marker_array);
  }
};

} // namespace as_training

RCLCPP_COMPONENTS_REGISTER_NODE(as_training::LidarPerceptionNode)