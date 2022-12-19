#include "utils.hpp"

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class FeatureExtraction : public rclcpp::Node {

public:

  FeatureExtraction() : Node("feature_extraction") {

    this->declare_parameter("cloud_msg_cache_size", 2);
    cloud_msg_cache_size = this->get_parameter("cloud_msg_cache_size").get_parameter_value().get<int>();

    this->declare_parameter("max_lidar_range", 30.0f);
    max_lidar_range = this->get_parameter("max_lidar_range").get_parameter_value().get<float>();

    this->declare_parameter("min_box_x", -0.3f);
    min_box_x = this->get_parameter("min_box_x").get_parameter_value().get<float>();
    this->declare_parameter("max_box_x", 0.8f);
    max_box_x = this->get_parameter("max_box_x").get_parameter_value().get<float>();

    this->declare_parameter("min_box_y", -0.25f);
    min_box_y = this->get_parameter("min_box_y").get_parameter_value().get<float>();
    this->declare_parameter("max_box_y", 0.25f);
    max_box_y = this->get_parameter("max_box_y").get_parameter_value().get<float>();

    this->declare_parameter("min_box_z", -0.3f);
    min_box_z = this->get_parameter("min_box_z").get_parameter_value().get<float>();
    this->declare_parameter("max_box_z", 0.3f);
    max_box_z = this->get_parameter("max_box_z").get_parameter_value().get<float>();

    this->declare_parameter("threshold_edge", 1.0f);
    threshold_edge = this->get_parameter("threshold_edge").get_parameter_value().get<float>();

    this->declare_parameter("threshold_flat", 0.1f);
    threshold_flat = this->get_parameter("threshold_flat").get_parameter_value().get<float>();

    this->declare_parameter("flat_leaf_size", 0.1f);
    flat_leaf_size = this->get_parameter("flat_leaf_size").get_parameter_value().get<float>();

    n = 0;

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, std::bind(&FeatureExtraction::cloudHandler, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<slam::msg::Cloud>("slam/cloud", 10);

    allocateMemory();

    resetCurrent();
  }

private:

  int n;

  // Parameters
  int cloud_msg_cache_size;
  float max_lidar_range;

  float min_box_x;
  float max_box_x;

  float min_box_y;
  float max_box_y;

  float min_box_z;
  float max_box_z;

  float threshold_edge;
  float threshold_flat;

  float flat_leaf_size;

  // Current
  sensor_msgs::msg::PointCloud2 current_cloud_msg;

  pcl::PointCloud<PointXYZIRT>::Ptr current_cloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr image_cloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr organized_cloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points;

  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_scan;
  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_scan_ds;

  std_msgs::msg::Header current_cloud_msg_header;

  std::vector<int> start_ring_index;
  std::vector<int> end_ring_index;

  std::vector<int> cloud_column_id;
  std::vector<float> cloud_range;

  std::vector<float> cloud_curvature;
  std::vector<smoothness_t> cloud_smoothness;
  std::vector<int> cloud_label;
  std::vector<int> cloud_neighbor_picked;

  long current_start_time_cloud;
  long current_end_time_cloud;

  float start_point_time;
  float end_point_time;

  cv::Mat mat_range;

  geometry_msgs::msg::TransformStamped t_start;
  geometry_msgs::msg::TransformStamped t_end;

  Eigen::Quaternionf rot_diff;
  Eigen::Quaternionf rot_base;
  Eigen::Vector3f trans_diff;

  Eigen::Quaternionf rot_start;
  Eigen::Quaternionf rot_end;

  Eigen::Vector3f trans_start;
  Eigen::Vector3f trans_end;


  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat;

  // Cloud msg cache
  std::deque<sensor_msgs::msg::PointCloud2> cloud_msg_cache;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  rclcpp::Publisher<slam::msg::Cloud>::SharedPtr publisher_;

  // tf2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void allocateMemory() {
    current_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
    image_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    organized_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points_scan.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points_scan_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());

    start_ring_index.resize(16);
    end_ring_index.resize(16);

    cloud_column_id.resize(16 * 1800);
    cloud_range.resize(16 * 1800);

    cloud_curvature.resize(16 * 1800);
    cloud_smoothness.resize(16 * 1800);
    cloud_label.resize(16 * 1800);
    cloud_neighbor_picked.resize(16 * 1800);

    voxel_grid_flat.setLeafSize(flat_leaf_size, flat_leaf_size, flat_leaf_size);
  }

  void resetCurrent() {
    current_cloud->clear();
    image_cloud->clear();
    organized_cloud->clear();

    edge_points->clear();
    flat_points->clear();
    flat_points_scan->clear();
    flat_points_scan_ds->clear();

    image_cloud->points.resize(16 * 1800);

    mat_range = cv::Mat(16, 1800, CV_32F, cv::Scalar::all(std::numeric_limits<float>::max()));
  }

  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    if (!cacheCloudMsg(cloud_msg)) { // If there is not enough point-cloud in cache
      return;
    }

    // Image projection

    imageProjection();

    organizeCloud();

    computeSmoothness();

    markUnreliable();

    exctractFeatures();

    publishClouds();

    resetCurrent();
  }

  bool cacheCloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    ++n;
    if (n <= 2) {  // TODO: Not useful but here the first cloud start before any odom. It just throws the new point clouds
      return false;
    }

    /*
    *  TODO: Check if this caching is usefull. The purpose is I think to have enough new odometry to be able to deskew it.
    *        We usually don't need to know instantaneously the robot pose.
    */
    cloud_msg_cache.push_back(*cloud_msg);  // Cache the point cloud received
    if ((int) cloud_msg_cache.size() <= cloud_msg_cache_size) {
      return false;
    }

    current_cloud_msg = std::move(cloud_msg_cache.front());  // std::move allows to copy and tell that we don't need the orignial copy anymore
    cloud_msg_cache.pop_front();

    pcl::moveFromROSMsg(current_cloud_msg, *current_cloud);

    current_cloud_msg_header = current_cloud_msg.header;

    current_start_time_cloud = rclcpp::Time(current_cloud_msg_header.stamp).nanoseconds() + int(current_cloud->points.front().time * 1e9); // TODO: See if we need nanoseconds
    current_end_time_cloud = rclcpp::Time(current_cloud_msg_header.stamp).nanoseconds() + int(current_cloud->points.back().time * 1e9);

    start_point_time = current_cloud->points.front().time;
    end_point_time = current_cloud->points.back().time;

    try {

      t_start = tf_buffer_->lookupTransform(
            "vision", "velodyne",
            rclcpp::Time(current_start_time_cloud));

      t_end = tf_buffer_->lookupTransform(
              "vision", "velodyne",
              rclcpp::Time(current_end_time_cloud));

    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not find transform : %s", ex.what());
      return false;
    }


    rot_start = Eigen::Quaternionf(t_start.transform.rotation.w, t_start.transform.rotation.x, t_start.transform.rotation.y, t_start.transform.rotation.z);
    rot_end = Eigen::Quaternionf(t_end.transform.rotation.w, t_end.transform.rotation.x, t_end.transform.rotation.y, t_end.transform.rotation.z);

    trans_start = Eigen::Vector3f(t_start.transform.translation.x, t_start.transform.translation.y, t_start.transform.translation.z);
    trans_end = Eigen::Vector3f(t_end.transform.translation.x, t_end.transform.translation.y, t_end.transform.translation.z);

    Eigen::Affine3f affine_start;
    affine_start.translation() = trans_start;
    affine_start.linear() = rot_start.toRotationMatrix();

    Eigen::Affine3f affine_end;
    affine_end.translation() = trans_end;
    affine_end.linear() = rot_end.toRotationMatrix();

    Eigen::Affine3f affine_diff = affine_start.inverse() * affine_end;

    rot_diff = affine_diff.linear();

    trans_diff = affine_diff.translation();

    Eigen::Vector3f v_diff = trans_diff / 0.1;

    float speed = v_diff.norm();

    RCLCPP_INFO(this->get_logger(), "Speed : %f", speed);

    Eigen::Matrix3f base_rot_matrix = Eigen::Matrix3f::Identity();
    rot_base = Eigen::Quaternionf(base_rot_matrix);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*current_cloud, *current_cloud, indices); // needed the include <pcl/filters/impl/filter.hpp>

    return true;
  }

  void imageProjection() {

    int current_cloud_size = current_cloud->points.size();
    for (int i = 0; i < current_cloud_size; ++i) {

      pcl::PointXYZI point;
      point.x = current_cloud->points[i].x;
      point.y = current_cloud->points[i].y;
      point.z = current_cloud->points[i].z;
      point.intensity = current_cloud->points[i].intensity;

      float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      
      // Range filtering
      if (range > max_lidar_range) {
        continue;
      }

      // Box filtering
      if ((point.x > min_box_x && point.x < max_box_x) && (point.y > min_box_y && point.y < max_box_y) && (point.z > min_box_z && point.z < max_box_z)) {
        continue;
      }

      int row_id = current_cloud->points[i].ring;

      if ((row_id < 0) || (row_id >= 16)) {
        continue;
      }

      float horizontal_angle = std::atan2(point.x, point.y) * 180 / M_PI;

      int column_id = -std::round((horizontal_angle - 90.0) / (360.0 / 1800.0)) + 1800 / 2;
      if (column_id >= 1800) {
        column_id -= 1800;
      }

      if ((column_id < 0) || (column_id >= 1800)) {
        continue;
      }

      if (mat_range.at<float>(row_id, column_id) != std::numeric_limits<float>::max()) {
        continue;
      }

      deskewPoint(point, current_cloud->points[i].time);

      mat_range.at<float>(row_id, column_id) = range;
      image_cloud->points[column_id + row_id * 1800] = point;
    }
  }

  void deskewPoint(pcl::PointXYZI & p, float point_time) {  // TODO: change this maybe

    // long current_point_time = current_start_time_cloud + int((point_time - start_point_time) * 1e9);

    // geometry_msgs::msg::TransformStamped t;
    // t = tf_buffer_->lookupTransform(
    //         "odom", "velodyne",
    //         rclcpp::Time(current_point_time));

    // Eigen::Quaternionf rot_point = Eigen::Quaternionf(t_start.transform.rotation.w, t_start.transform.rotation.x, t_start.transform.rotation.y, t_start.transform.rotation.z);

    // Eigen::Vector3f trans_point = Eigen::Vector3f(t_start.transform.translation.x, t_start.transform.translation.y, t_start.transform.translation.z);

    // Eigen::Affine3f affine_point;
    // affine_point.translation() = trans_point;
    // affine_point.linear() = rot_point.toRotationMatrix();

    // Eigen::Affine3f affine_start;
    // affine_start.translation() = trans_start;
    // affine_start.linear() = rot_start.toRotationMatrix();

    // Eigen::Vector3f vec_point(p.x, p.y, p.z);
    // Eigen::Vector3f vec_point_orig(p.x, p.y, p.z);
    // vec_point = ( affine_start.inverse() * affine_point ) * vec_point;

    // RCLCPP_INFO(this->get_logger(), "Correction : %f, %f, %f", vec_point(0) - vec_point_orig(0), vec_point(1) - vec_point_orig(1), vec_point(2) - vec_point_orig(2));

    float alpha = (point_time - start_point_time) / (end_point_time - start_point_time);

    Eigen::Affine3f affine_correction;
    affine_correction.translation() = (1.0 - alpha) * trans_start + alpha * trans_end;
    affine_correction.linear() = rot_start.slerp(alpha, rot_end).toRotationMatrix();
    // Eigen::Matrix4f trans_correction = affine_correction.inverse().matrix();

    Eigen::Affine3f affine_start;
    affine_start.translation() = trans_start;
    affine_start.linear() = rot_start.toRotationMatrix();

    Eigen::Vector3f vec_point(p.x, p.y, p.z);
    Eigen::Vector3f vec_point_orig(p.x, p.y, p.z);
    vec_point = ( affine_start.inverse() * affine_correction ) * vec_point;

    // RCLCPP_INFO(this->get_logger(), "Correction : %f, %f, %f", vec_point(0) - vec_point_orig(0), vec_point(1) - vec_point_orig(1), vec_point(2) - vec_point_orig(2));

    p.x = vec_point(0);
    p.y = vec_point(1);
    p.z = vec_point(2);
  }

  void organizeCloud() {

    int count = 0;

    for (int i = 0; i < 16; ++i) {

      start_ring_index[i] = count - 1 + 5;

      for (int j = 0; j < 1800; ++j) {

        if (mat_range.at<float>(i, j) != std::numeric_limits<float>::max()) {

          cloud_column_id[count] = j;

          cloud_range[count] = mat_range.at<float>(i, j);

          organized_cloud->push_back(image_cloud->points[j + i * 1800]);

          ++count;
        }

      }

      end_ring_index[i] = count - 1 - 5;

    }
  }

  void computeSmoothness() {

    int cloud_size = (int) organized_cloud->points.size();

    for (int i = 5; i < cloud_size - 5; ++i) {

      float diff_range = cloud_range[i - 5] + cloud_range[i - 4] 
                       + cloud_range[i - 3] + cloud_range[i - 2] 
                       + cloud_range[i - 1] - 10 * cloud_range[i]
                       + cloud_range[i + 1] + cloud_range[i + 2]
                       + cloud_range[i + 3] + cloud_range[i + 4]
                       + cloud_range[i + 5];

      cloud_curvature[i] = diff_range * diff_range;

      cloud_label[i] = 0;
      cloud_neighbor_picked[i] = 0;

      cloud_smoothness[i].value = cloud_curvature[i];
      cloud_smoothness[i].ind = i;
    }

  }

  void markUnreliable() {

    int cloud_size = (int) organized_cloud->points.size();

    for (int i = 5; i < cloud_size - 6; ++i) {

      float range_1 = cloud_range[i];
      float range_2 = cloud_range[i + 1];
      int column_diff = std::abs(int(cloud_column_id[i + 1] - cloud_column_id[i]));  // Check if the int() call is usefull

      // Occluded points
      if (column_diff < 10) {

        if (range_1 - range_2 > 0.3) {

          cloud_neighbor_picked[i - 5] = 1;
          cloud_neighbor_picked[i - 4] = 1;
          cloud_neighbor_picked[i - 3] = 1;
          cloud_neighbor_picked[i - 2] = 1;
          cloud_neighbor_picked[i - 1] = 1;
          cloud_neighbor_picked[i] = 1;

        } else if (range_2 - range_1 > 0.3) {

          cloud_neighbor_picked[i + 1] = 1;
          cloud_neighbor_picked[i + 2] = 1;
          cloud_neighbor_picked[i + 3] = 1;
          cloud_neighbor_picked[i + 4] = 1;
          cloud_neighbor_picked[i + 5] = 1;
          cloud_neighbor_picked[i + 6] = 1;

        }

      }

      // Parallel beam

      float diff_1 = std::abs(float(cloud_range[i - 1] - cloud_range[i]));
      float diff_2 = std::abs(float(cloud_range[i] - cloud_range[i + 1]));

      if (diff_1 > 0.02 * cloud_range[i] && diff_2 > 0.02 * cloud_range[i]) {
        cloud_neighbor_picked[i] = 1;
      }

    }

  }

  void exctractFeatures() {

    for (int i = 0; i < 16; ++i) {

      flat_points_scan->clear();

      for (int j = 0; j < 6; ++j) {

        int sp = (start_ring_index[i] * (6 - j) + end_ring_index[i] * j) / 6;
        int ep = (start_ring_index[i] * (5 - j) + end_ring_index[i] * (j + 1)) / 6 - 1;

        if (sp >= ep) {
          continue;
        }

        std::sort(cloud_smoothness.begin() + sp, cloud_smoothness.begin() + ep, by_value());

        int n_edge_points = 0;
        for (int k = ep; k >= sp; --k) {

          int idx = cloud_smoothness[k].ind;

          if (cloud_neighbor_picked[idx] == 0 && cloud_curvature[idx] > threshold_edge) {

            ++n_edge_points;
            if (n_edge_points <= 20) {
              cloud_label[idx] = 1;
              edge_points->push_back(organized_cloud->points[idx]);
            } else {
              break;
            }

            cloud_neighbor_picked[idx] = 1;
            for (int l = 1; l <= 5; ++l) {
              int column_diff = std::abs(int(cloud_column_id[idx + l] - cloud_column_id[idx + l - 1]));
              if (column_diff > 10) {
                break;
              }
              cloud_neighbor_picked[idx + l] = 1;
            }
            for (int l = -1; l >= -5; --l) {
              int column_diff = std::abs(int(cloud_column_id[idx + l] - cloud_column_id[idx + l + 1]));
              if (column_diff > 10) {
                break;
              }
              cloud_neighbor_picked[idx + l] = 1;
            }
          }
        }

        for (int k = sp; k <= ep; ++k) {

          int idx = cloud_smoothness[k].ind;
          if (cloud_neighbor_picked[idx] == 0 && cloud_curvature[idx] < threshold_flat) {

            cloud_label[idx] = -1;
            cloud_neighbor_picked[idx] = 1;

            for (int l = 1; l <= 5; ++l) {
              int column_diff = std::abs(int(cloud_column_id[idx + l] - cloud_column_id[idx + l - 1]));
              if (column_diff > 10) {
                break;
              }
              cloud_neighbor_picked[idx + l] = 1;
            }
            for (int l = -1; l >= -5; --l) {
              int column_diff = std::abs(int(cloud_column_id[idx + l] - cloud_column_id[idx + l + 1]));
              if (column_diff > 10) {
                break;
              }
              cloud_neighbor_picked[idx + l] = 1;
            }
          }

        }

        for (int k = sp; k <= ep; ++k) {
          if (cloud_label[k] <= 0) { // TODO : Experiment with the <= 0 ( < 0 instead ). Here I don't see why making the previous code if we select everything anyway. May need to remove the degenerate part.
            flat_points_scan->push_back(organized_cloud->points[k]);
          }
        }

        flat_points_scan_ds->clear();
        voxel_grid_flat.setInputCloud(flat_points_scan);
        voxel_grid_flat.setLeafSize(0.1, 0.1, 0.1);
        voxel_grid_flat.filter(*flat_points_scan_ds);

        *flat_points += *flat_points_scan_ds;
      }
    }
  }

  void publishClouds() {

    sensor_msgs::msg::PointCloud2 temp_edge_points;
    sensor_msgs::msg::PointCloud2 temp_flat_points;
    sensor_msgs::msg::PointCloud2 temp_all_points;

    pcl::toROSMsg(*edge_points, temp_edge_points);
    pcl::toROSMsg(*flat_points, temp_flat_points);
    pcl::toROSMsg(*organized_cloud, temp_all_points);

    slam::msg::Cloud cloud_msg;

    cloud_msg.cloud_deskewed = temp_all_points;
    cloud_msg.cloud_edge = temp_edge_points;
    cloud_msg.cloud_flat = temp_flat_points;

    RCLCPP_INFO(this->get_logger(), "Size %d", cloud_msg.cloud_deskewed.width * cloud_msg.cloud_deskewed.height);

    cloud_msg.header = current_cloud_msg_header;

    // geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
    //         "vision", "velodyne",
    //         rclcpp::Time(current_start_time_cloud));

    // cloud_msg.initial_guess_x = t.transform.translation.x;
    // cloud_msg.initial_guess_y = t.transform.translation.y;
    // cloud_msg.initial_guess_z = t.transform.translation.z;
    // cloud_msg.initial_guess_rot_w = t.transform.rotation.w;
    // cloud_msg.initial_guess_rot_x = t.transform.rotation.x;
    // cloud_msg.initial_guess_rot_y = t.transform.rotation.y;
    // cloud_msg.initial_guess_rot_z = t.transform.rotation.z;

    cloud_msg.initial_guess_x = t_start.transform.translation.x;
    cloud_msg.initial_guess_y = t_start.transform.translation.y;
    cloud_msg.initial_guess_z = t_start.transform.translation.z;
    cloud_msg.initial_guess_rot_w = t_start.transform.rotation.w;
    cloud_msg.initial_guess_rot_x = t_start.transform.rotation.x;
    cloud_msg.initial_guess_rot_y = t_start.transform.rotation.y;
    cloud_msg.initial_guess_rot_z = t_start.transform.rotation.z;

    publisher_->publish(cloud_msg);
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FeatureExtraction>());
  rclcpp::shutdown();
  return 0;
}