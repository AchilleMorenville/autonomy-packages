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

    this->declare_parameter("body_tform_velodyne_x", -0.202f);
    body_tform_velodyne_x = this->get_parameter("body_tform_velodyne_x").get_parameter_value().get<float>();

    this->declare_parameter("body_tform_velodyne_y", 0.0f);
    body_tform_velodyne_y = this->get_parameter("body_tform_velodyne_y").get_parameter_value().get<float>();

    this->declare_parameter("body_tform_velodyne_z", 0.151f);
    body_tform_velodyne_z = this->get_parameter("body_tform_velodyne_z").get_parameter_value().get<float>();

    this->declare_parameter("body_tform_velodyne_rot_x", -0.0f);
    body_tform_velodyne_rot_x = this->get_parameter("body_tform_velodyne_rot_x").get_parameter_value().get<float>();

    this->declare_parameter("body_tform_velodyne_rot_y", 0.0f);
    body_tform_velodyne_rot_y = this->get_parameter("body_tform_velodyne_rot_y").get_parameter_value().get<float>();

    this->declare_parameter("body_tform_velodyne_rot_z", -0.0f);
    body_tform_velodyne_rot_z = this->get_parameter("body_tform_velodyne_rot_z").get_parameter_value().get<float>();

    this->declare_parameter("body_tform_velodyne_rot_w", 1.0f);
    body_tform_velodyne_rot_w = this->get_parameter("body_tform_velodyne_rot_w").get_parameter_value().get<float>();    


    n = 0;

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setUsingDedicatedThread(true);

    // tf_listener_ =
    //   std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    callback_group_odom = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions odom_options = rclcpp::SubscriptionOptions();
    odom_options.callback_group = callback_group_odom;

    cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 1000, std::bind(&FeatureExtraction::cloudHandler, this, std::placeholders::_1)
    );

    vo_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "spot_driver/odometry/vo_odom", 1000, std::bind(&FeatureExtraction::odomHandler, this, std::placeholders::_1), odom_options
    );



    publisher_ = this->create_publisher<slam::msg::Cloud>("slam/features", 10);

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

  float body_tform_velodyne_x;
  float body_tform_velodyne_y;
  float body_tform_velodyne_z;
  float body_tform_velodyne_rot_x;
  float body_tform_velodyne_rot_y;
  float body_tform_velodyne_rot_z;
  float body_tform_velodyne_rot_w;

  Eigen::Matrix4f body_tform_velodyne;

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

  Eigen::Affine3f affine_start;
  Eigen::Affine3f affine_end;


  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat;

  pcl::PointCloud<pcl::PointXYZ>::Ptr odom_cloud;

  // Cloud msg cache
  std::deque<sensor_msgs::msg::PointCloud2> cloud_msg_cache;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr vo_subscription_;
  rclcpp::CallbackGroup::SharedPtr callback_group_odom;

  rclcpp::Publisher<slam::msg::Cloud>::SharedPtr publisher_;

  // tf2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::mutex tf_buffer_mtx;

  void allocateMemory() {
    current_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
    image_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    organized_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

    edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points_scan.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points_scan_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());

    odom_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());

    start_ring_index.resize(16);
    end_ring_index.resize(16);

    cloud_column_id.resize(16 * 1800);
    cloud_range.resize(16 * 1800);

    cloud_curvature.resize(16 * 1800);
    cloud_smoothness.resize(16 * 1800);
    cloud_label.resize(16 * 1800);
    cloud_neighbor_picked.resize(16 * 1800);

    voxel_grid_flat.setLeafSize(flat_leaf_size, flat_leaf_size, flat_leaf_size);

    body_tform_velodyne = valuesToMatrix(
      body_tform_velodyne_x,
      body_tform_velodyne_y,
      body_tform_velodyne_z,
      body_tform_velodyne_rot_x,
      body_tform_velodyne_rot_y,
      body_tform_velodyne_rot_z,
      body_tform_velodyne_rot_w
    );

    geometry_msgs::msg::TransformStamped tf_velodyne;
    tf_velodyne.header.frame_id = "body";
    tf_velodyne.child_frame_id = "velodyne";
    tf_velodyne.transform.translation.x = body_tform_velodyne_x;
    tf_velodyne.transform.translation.y = body_tform_velodyne_y;
    tf_velodyne.transform.translation.z = body_tform_velodyne_z;
    tf_velodyne.transform.rotation.x = body_tform_velodyne_rot_x;
    tf_velodyne.transform.rotation.y = body_tform_velodyne_rot_y;
    tf_velodyne.transform.rotation.z = body_tform_velodyne_rot_z;
    tf_velodyne.transform.rotation.w = body_tform_velodyne_rot_w;

    {
      std::lock_guard<std::mutex> lock(tf_buffer_mtx);
      tf_buffer_->setTransform(
          tf_velodyne, "transform_odometry", true);
    }
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

  void odomHandler(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header = odom_msg->header;
    tf_stamped.child_frame_id = odom_msg->child_frame_id;
    tf_stamped.transform.translation.x = odom_msg->pose.pose.position.x;
    tf_stamped.transform.translation.y = odom_msg->pose.pose.position.y;
    tf_stamped.transform.translation.z = odom_msg->pose.pose.position.z;
    tf_stamped.transform.rotation = odom_msg->pose.pose.orientation;
    {
      std::lock_guard<std::mutex> lock(tf_buffer_mtx);
      tf_buffer_->setTransform(
          tf_stamped, "transform_odometry", false);
    }
  }

  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    if (!cacheCloudMsg(cloud_msg)) { // If there is not enough point-cloud in cache
      return;
    }

    // Image projection
    {

      Timer timer;
      imageProjection();

      organizeCloud();

      computeSmoothness();

      markUnreliable();

      exctractFeatures();

      publishClouds();

      resetCurrent();
    }
  }

  bool cacheCloudMsg(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    RCLCPP_INFO(this->get_logger(), "Received cloud : %d", n);

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

    current_start_time_cloud = rclcpp::Time(current_cloud_msg_header.stamp).nanoseconds() + long(current_cloud->points.front().time * 1e9); // TODO: See if we need nanoseconds
    current_end_time_cloud = rclcpp::Time(current_cloud_msg_header.stamp).nanoseconds() + long(current_cloud->points.back().time * 1e9);

    start_point_time = current_cloud->points.front().time;
    end_point_time = current_cloud->points.back().time;

    RCLCPP_INFO(this->get_logger(), "Ça bloque ici");

    {
      std::lock_guard<std::mutex> lock(tf_buffer_mtx);
      if (tf_buffer_->canTransform("vision", "body", rclcpp::Time(current_start_time_cloud))) {

        t_start = tf_buffer_->lookupTransform("vision", "body", rclcpp::Time(current_start_time_cloud));

      } else {
        RCLCPP_INFO(this->get_logger(), "Could not find transform vision_tform_body");
        return false;
      }

      if (tf_buffer_->canTransform("vision", "body", rclcpp::Time(current_end_time_cloud))) {

        t_end = tf_buffer_->lookupTransform("vision", "body", rclcpp::Time(current_end_time_cloud));

      } else {
        RCLCPP_INFO(this->get_logger(), "Could not find transform vision_tform_body");
        return false;
      }
    }

    rot_start = Eigen::Quaternionf(t_start.transform.rotation.w, t_start.transform.rotation.x, t_start.transform.rotation.y, t_start.transform.rotation.z);
    rot_end = Eigen::Quaternionf(t_end.transform.rotation.w, t_end.transform.rotation.x, t_end.transform.rotation.y, t_end.transform.rotation.z);

    trans_start = Eigen::Vector3f(t_start.transform.translation.x, t_start.transform.translation.y, t_start.transform.translation.z);
    trans_end = Eigen::Vector3f(t_end.transform.translation.x, t_end.transform.translation.y, t_end.transform.translation.z);

    affine_start.translation() = trans_start;
    affine_start.linear() = rot_start.toRotationMatrix();

    affine_end.translation() = trans_end;
    affine_end.linear() = rot_end.toRotationMatrix();

    Eigen::Affine3f affine_body_tform_velodyne;
    affine_body_tform_velodyne.matrix() = body_tform_velodyne;

    affine_start = affine_start * affine_body_tform_velodyne;
    affine_end = affine_end * affine_body_tform_velodyne;

    rot_start = affine_start.linear();
    rot_end = affine_end.linear();

    trans_start = affine_start.translation();
    trans_end = affine_end.translation();

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
    float alpha = (point_time - start_point_time) / (end_point_time - start_point_time);

    Eigen::Affine3f affine_correction;
    affine_correction.translation() = (1.0 - alpha) * trans_start + alpha * trans_end;
    affine_correction.linear() = rot_start.slerp(alpha, rot_end).toRotationMatrix();

    Eigen::Vector3f vec_point(p.x, p.y, p.z);
    Eigen::Vector3f vec_point_orig(p.x, p.y, p.z);
    vec_point = ( affine_start.inverse() * affine_correction ) * vec_point;

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
          if (cloud_label[k] < 0) { // TODO : Experiment with the <= 0 ( < 0 instead ). Here I don't see why making the previous code if we select everything anyway. May need to remove the degenerate part.
            flat_points_scan->push_back(organized_cloud->points[k]);
          }
        }

        // flat_points_scan_ds->clear();
        // voxel_grid_flat.setInputCloud(flat_points_scan);
        // voxel_grid_flat.setLeafSize(0.1, 0.1, 0.1);
        // voxel_grid_flat.filter(*flat_points_scan_ds);

        // *flat_points += *flat_points_scan_ds;

        *flat_points += *flat_points_scan;
      }
    }
  }

  void publishClouds() {

    sensor_msgs::msg::PointCloud2 temp_edge_points;
    sensor_msgs::msg::PointCloud2 temp_flat_points;
    sensor_msgs::msg::PointCloud2 temp_all_points;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_edge_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_flat_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_organized_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = 0; i < organized_cloud->points.size(); ++i) {

      pcl::PointXYZI point;
      point.x = organized_cloud->points[i].x;
      point.y = organized_cloud->points[i].y;
      point.z = organized_cloud->points[i].z;
      point.intensity = organized_cloud->points[i].intensity;

      float range = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);

      if (range > 10) {
        continue;
      }

      filtered_organized_cloud->push_back(point);

    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_organized_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::transformPointCloud(*edge_points, *transformed_edge_points, body_tform_velodyne);
    pcl::transformPointCloud(*flat_points, *transformed_flat_points, body_tform_velodyne);
    pcl::transformPointCloud(*filtered_organized_cloud, *transformed_organized_cloud, body_tform_velodyne);

    pcl::toROSMsg(*transformed_edge_points, temp_edge_points);
    pcl::toROSMsg(*transformed_flat_points, temp_flat_points);
    pcl::toROSMsg(*transformed_organized_cloud, temp_all_points);

    slam::msg::Cloud cloud_msg;

    cloud_msg.cloud_deskewed = temp_all_points;
    cloud_msg.cloud_edge = temp_edge_points;
    cloud_msg.cloud_flat = temp_flat_points;

    // RCLCPP_INFO(this->get_logger(), "Size %d", cloud_msg.cloud_deskewed.width * cloud_msg.cloud_deskewed.height);

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

    // cloud_msg.initial_guess_x = trans_start(0);
    // cloud_msg.initial_guess_y = trans_start(1);
    // cloud_msg.initial_guess_z = trans_start(2);
    // cloud_msg.initial_guess_rot_w = rot_start.w();
    // cloud_msg.initial_guess_rot_x = rot_start.x();
    // cloud_msg.initial_guess_rot_y = rot_start.y();
    // cloud_msg.initial_guess_rot_z = rot_start.z();

    cloud_msg.initial_guess_x = t_start.transform.translation.x;
    cloud_msg.initial_guess_y = t_start.transform.translation.y;
    cloud_msg.initial_guess_z = t_start.transform.translation.z;
    cloud_msg.initial_guess_rot_w = t_start.transform.rotation.w;
    cloud_msg.initial_guess_rot_x = t_start.transform.rotation.x;
    cloud_msg.initial_guess_rot_y = t_start.transform.rotation.y;
    cloud_msg.initial_guess_rot_z = t_start.transform.rotation.z;

    publisher_->publish(cloud_msg);

    RCLCPP_INFO(this->get_logger(), "Cloud published");

    // odom_cloud->push_back(pcl::PointXYZ(t_start.transform.translation.x, t_start.transform.translation.y, t_start.transform.translation.z));

    // odom_cloud->push_back(pcl::PointXYZ(trans_start(0), trans_start(1), trans_start(2)));

    // if (n % 100 == 0) {

    //   pcl::io::savePCDFileASCII(std::string("/ros2_ws/data/path.pcd"), *odom_cloud);

    // }
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500));
  // rclcpp::executors::SingleThreadedExecutor exec;
  auto feature_extraction_node = std::make_shared<FeatureExtraction>();
  exec.add_node(feature_extraction_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Feature Extraction Started.\033[0m");

  exec.spin();
  rclcpp::shutdown();
  return 0;
}