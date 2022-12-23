#include "utils.hpp"
#include "optimization.hpp"

struct smoothness_t{ 
    float value;
    size_t ind;
};

struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;
    }
};

class LidarOdometry : public rclcpp::Node {

public:

  LidarOdometry() : Node("lidar_odometry") {

    n = 0;

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

    this->declare_parameter("flat_leaf_size", 0.2f);
    flat_leaf_size = this->get_parameter("flat_leaf_size").get_parameter_value().get<float>();

    this->declare_parameter("edge_leaf_size", 0.1f);
    edge_leaf_size = this->get_parameter("edge_leaf_size").get_parameter_value().get<float>();

    cloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "velodyne_points", 10, std::bind(&LidarOdometry::cloudHandler, this, std::placeholders::_1)
    );

    allocateMemory();

    resetCurrent();
  }

private:

  int n;

  // Parameters
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
  float edge_leaf_size;

  // Current
  sensor_msgs::msg::PointCloud2 current_cloud_msg;
  std_msgs::msg::Header current_cloud_msg_header;

  pcl::PointCloud<PointXYZIRT>::Ptr current_cloud;

  long current_start_time_cloud;
  long current_end_time_cloud;

  float start_point_time;
  float end_point_time;

  cv::Mat mat_range;

  pcl::PointCloud<pcl::PointXYZI>::Ptr image_cloud;

  pcl::PointCloud<pcl::PointXYZI>::Ptr organized_cloud;

  std::vector<int> start_ring_index;
  std::vector<int> end_ring_index;

  std::vector<int> cloud_column_id;
  std::vector<float> cloud_range;

  std::vector<float> cloud_curvature;
  std::vector<smoothness_t> cloud_smoothness;
  std::vector<int> cloud_label;
  std::vector<int> cloud_neighbor_picked;

  pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points;

  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_scan;

  pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_scan_ds;

  // Last

  Eigen::Matrix4f displacement_estimation;

  Eigen::Matrix4f last_lidar_odometry;

  // Optimization

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_edge;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat;

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat_features;

  Eigen::Matrix4f optimized_lidar_odometry;

  // Frames

  pcl::PointCloud<pcl::PointXYZL>::Ptr poses_3D;

  // std::unordered_map<int, Eigen::Matrix4f> poses_6D;
  // std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> poses_edge_points;
  // std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> poses_flat_points;

  std::vector<Eigen::Matrix4f> poses_6D;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> poses_edge_points;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> poses_flat_points;

  // Local map

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_flat_points;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_edge_points;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_flat_points;

  pcl::CropBox<pcl::PointXYZL> boxFilter;

  // All map

  pcl::PointCloud<pcl::PointXYZI>::Ptr map;

  // Subscription
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;

  void allocateMemory() {
    current_cloud.reset(new pcl::PointCloud<PointXYZIRT>());
    image_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points_scan_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());

    displacement_estimation = Eigen::Matrix4f::Identity();
    last_lidar_odometry = Eigen::Matrix4f::Identity();
    optimized_lidar_odometry = Eigen::Matrix4f::Identity();

    organized_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    start_ring_index.resize(16);
    end_ring_index.resize(16);

    cloud_column_id.resize(16 * 1800);
    cloud_range.resize(16 * 1800);

    cloud_curvature.resize(16 * 1800);
    cloud_smoothness.resize(16 * 1800);
    cloud_label.resize(16 * 1800);
    cloud_neighbor_picked.resize(16 * 1800);

    edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    flat_points_scan.reset(new pcl::PointCloud<pcl::PointXYZI>());

    poses_3D.reset(new pcl::PointCloud<pcl::PointXYZL>());

    voxel_grid_flat.setLeafSize(flat_leaf_size, flat_leaf_size, flat_leaf_size);
    voxel_grid_edge.setLeafSize(edge_leaf_size, edge_leaf_size, edge_leaf_size);
    voxel_grid_flat_features.setLeafSize(0.1, 0.1, 0.1);

    local_map_edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
    local_map_flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>);

    kdtree_local_map_edge_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_local_map_flat_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);

    boxFilter.setMin(Eigen::Vector4f(-10.0, -10.0, -10.0, 1.0));
    boxFilter.setMax(Eigen::Vector4f(10.0, 10.0, 10.0, 1));

    map.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  void resetCurrent() {
    current_cloud->clear();
    image_cloud->clear();

    organized_cloud->clear();

    image_cloud->points.resize(16 * 1800);
    mat_range = cv::Mat(16, 1800, CV_32F, cv::Scalar::all(std::numeric_limits<float>::max()));

    edge_points->clear();
    flat_points->clear();
    flat_points_scan->clear();
  }

  void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {

    RCLCPP_INFO(this->get_logger(), "Recieved : %d", n);

    // Save current input
    current_cloud_msg = *cloud_msg;

    pcl::moveFromROSMsg(current_cloud_msg, *current_cloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*current_cloud, *current_cloud, indices); // needed the include <pcl/filters/impl/filter.hpp>

    current_cloud_msg_header = current_cloud_msg.header;

    current_start_time_cloud = rclcpp::Time(current_cloud_msg_header.stamp).nanoseconds() + int(current_cloud->points.front().time * 1e9); // TODO: See if we need nanoseconds
    current_end_time_cloud = rclcpp::Time(current_cloud_msg_header.stamp).nanoseconds() + int(current_cloud->points.back().time * 1e9);

    start_point_time = current_cloud->points.front().time;
    end_point_time = current_cloud->points.back().time;

    // RCLCPP_INFO(this->get_logger(), "Start time : %f", start_point_time);
    // RCLCPP_INFO(this->get_logger(), "End time : %f", end_point_time);
    // RCLCPP_INFO(this->get_logger(), "Delta : %f", end_point_time);

    {
      Timer timer;

      // Feature extraction and initial deskewing

      imageProjection();

      organizeCloud();

      computeSmoothness();

      markUnreliable();

      exctractFeatures();

    }

    {

      Timer timer;

      // Optimization

      optimizeCurrent();

    }

    {
      Timer timer;

      // Save current

      saveCurrent(); // Final deskewing if necessary

    }

    {
      Timer timer;
      // Local map creation

      computeLocalMap();

      resetCurrent();

      ++n;

    }

  }

  void imageProjection() {

    int current_cloud_size = current_cloud->points.size();
    for (int i = 0; i < current_cloud_size; ++i) {

      pcl::PointXYZI point;
      point.x = current_cloud->points[i].x;
      point.y = current_cloud->points[i].y;
      point.z = current_cloud->points[i].z;
      point.intensity = current_cloud->points[i].time;  // Don't need intensity for now so save time

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
      
      // deskewPoint(point, displacement_estimation, current_cloud->points[i].time);

      // TODO: Check if points are well distorted


      // Compute real start and end point time
      if (point.intensity < start_point_time) {
        start_point_time = point.intensity;
      }
      if (point.intensity > end_point_time) {
        end_point_time = point.intensity;
      }

      mat_range.at<float>(row_id, column_id) = range;
      image_cloud->points[column_id + row_id * 1800] = point;
    }
  }

  void deskewPoint(pcl::PointXYZI & p, Eigen::Matrix4f transformation, float point_time) {

    float alpha = (point_time - start_point_time) / (end_point_time - start_point_time);

    // Eigen::Affine3f affine_correction;
    // affine_correction.translation() = alpha * transformation.block<3, 1>(0, 3);

    // Eigen::Quaternionf quat_transformation;
    // quat_transformation = transformation.block<3, 3>(0, 0);

    // Eigen::Quaternionf quat_identity;
    // quat_identity = Eigen::Matrix3f::Identity();
    // // Eigen::Quaternionf quat_identity(1, 0, 0, 0);

    // affine_correction.linear() = quat_identity.slerp(alpha, quat_transformation).toRotationMatrix();

    // Eigen::Vector3f vec_point(p.x, p.y, p.z);
    // vec_point = affine_correction * vec_point;

    if (alpha < 0 || alpha > 1.001) {
      RCLCPP_INFO(this->get_logger(), "Wrong ratio : %f", alpha);
    }

    Eigen::Vector3f vec_point(p.x, p.y, p.z);

    vec_point(0) = p.x - alpha * transformation(0, 3);
    vec_point(1) = p.y - alpha * transformation(1, 3);
    vec_point(2) = p.z - alpha * transformation(2, 3);

    Eigen::Quaternionf q_id, q_es, q_f;
    q_es = transformation.block<3, 3>(0, 0);
    q_id.setIdentity();
    q_f = q_id.slerp(alpha, q_es);

    vec_point = q_f.conjugate() * vec_point;

    p.x = vec_point(0);
    p.y = vec_point(1);
    p.z = vec_point(2);
  }

  void deskewPointEnd(pcl::PointXYZI & p, Eigen::Matrix4f displacement) {

    float alpha = (end_point_time - p.intensity) / (end_point_time - start_point_time);

    // Eigen::Affine3f affine_correction;
    // affine_correction.translation() = alpha * transformation.block<3, 1>(0, 3);

    // Eigen::Quaternionf quat_transformation;
    // quat_transformation = transformation.block<3, 3>(0, 0);

    // Eigen::Quaternionf quat_identity;
    // quat_identity = Eigen::Matrix3f::Identity();
    // // Eigen::Quaternionf quat_identity(1, 0, 0, 0);

    // affine_correction.linear() = quat_identity.slerp(alpha, quat_transformation).toRotationMatrix();

    // Eigen::Vector3f vec_point(p.x, p.y, p.z);
    // vec_point = affine_correction * vec_point;

    if (alpha < 0 || alpha > 1.001) {
      RCLCPP_INFO(this->get_logger(), "Wrong ratio : %f", alpha);
    }

    Eigen::Vector3f vec_point(p.x, p.y, p.z);
    Eigen::Matrix3f rot_T = displacement.block<3, 3>(0, 0).transpose();

    Eigen::Quaternionf q_identity, q_displacement, q_final;
    q_displacement = rot_T;
    q_identity.setIdentity();
    q_final = q_identity.slerp(alpha, q_displacement);

    vec_point = q_final * vec_point + alpha * (- rot_T * displacement.block<3, 1>(0, 3));

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
        voxel_grid_flat_features.setInputCloud(flat_points_scan);
        // voxel_grid_flat_features.setLeafSize(0.1, 0.1, 0.1);
        voxel_grid_flat_features.filter(*flat_points_scan_ds);

        *flat_points += *flat_points_scan_ds;

        // Keep everything for now as we usually still voxel grid afterward. It is to avoir making means of means
        // *flat_points += *flat_points_scan;
      }
    }
  }

  void optimizeCurrent() {

    if (poses_3D->points.empty() == true) { // If there is no optimization to be done
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points_ds(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_ds(new pcl::PointCloud<pcl::PointXYZI>());

    voxel_grid_edge.setInputCloud(edge_points);
    voxel_grid_edge.filter(*edge_points_ds);

    voxel_grid_flat.setInputCloud(flat_points);
    voxel_grid_flat.filter(*flat_points_ds);

    RCLCPP_INFO(this->get_logger(), "Nbr flat points : %d", flat_points_ds->points.size());
    RCLCPP_INFO(this->get_logger(), "Nbr edge points : %d", edge_points_ds->points.size());

    pcl::PointCloud<pcl::PointXYZI>::Ptr deskewed_edge_points_ds(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr deskewed_flat_points_ds(new pcl::PointCloud<pcl::PointXYZI>());

    int size_edge_points_ds = (int) edge_points_ds->points.size();
    for (int i = 0; i < size_edge_points_ds; ++i) {
      pcl::PointXYZI point = edge_points_ds->points[i];
      deskewPointEnd(point, displacement_estimation);
      deskewed_edge_points_ds->push_back(point);
    }

    int size_flat_points_ds = (int) flat_points_ds->points.size();
    for (int i = 0; i < size_flat_points_ds; ++i) {
      pcl::PointXYZI point = flat_points_ds->points[i];
      deskewPointEnd(point, displacement_estimation);
      deskewed_flat_points_ds->push_back(point);
    }


    // Estimated pose using the estimated displacement and odometry
    Eigen::Matrix4f estimated_lidar_odometry = last_lidar_odometry * displacement_estimation;

    optimized_lidar_odometry = optimize(
      estimated_lidar_odometry,
      deskewed_edge_points_ds,
      deskewed_flat_points_ds,
      local_map_edge_points,
      local_map_flat_points,
      kdtree_local_map_edge_points,
      kdtree_local_map_flat_points,
      0.20
    );

  }

  void saveCurrent() {
    displacement_estimation = getDifferenceTransformation(last_lidar_odometry, optimized_lidar_odometry);
    last_lidar_odometry = optimized_lidar_odometry;

    pcl::PointXYZL point;
    point.x = optimized_lidar_odometry(0, 3);
    point.y = optimized_lidar_odometry(1, 3);
    point.z = optimized_lidar_odometry(2, 3);
    point.label = n;

    poses_3D->push_back(point);

    int size_edge_points = (int) edge_points->points.size();
    for (int i = 0; i < size_edge_points; ++i) {
      pcl::PointXYZI point = edge_points->points[i];
      deskewPointEnd(point, displacement_estimation);
      edge_points->points[i] = point;
    }

    int size_flat_points = (int) flat_points->points.size();
    for (int i = 0; i < size_flat_points; ++i) {
      pcl::PointXYZI point = flat_points->points[i];
      deskewPointEnd(point, displacement_estimation);
      flat_points->points[i] = point;
    }

    int size_organized_cloud = (int) organized_cloud->points.size();
    for (int i = 0; i < size_organized_cloud; ++i) {
      pcl::PointXYZI point = organized_cloud->points[i];
      deskewPointEnd(point, displacement_estimation);
      organized_cloud->points[i] = point;
    }



    pcl::PointCloud<pcl::PointXYZI>::Ptr organized_cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*organized_cloud, *organized_cloud_transformed, optimized_lidar_odometry);

    *map += *organized_cloud_transformed;

    // TODO: Final deskewing

    pcl::PointCloud<pcl::PointXYZI>::Ptr copy_edge_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr copy_flat_points(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::copyPointCloud(*edge_points,  *copy_edge_points);
    pcl::copyPointCloud(*flat_points,  *copy_flat_points);

    pcl::transformPointCloud(*copy_edge_points, *copy_edge_points, optimized_lidar_odometry);
    pcl::transformPointCloud(*copy_flat_points, *copy_flat_points, optimized_lidar_odometry);

    // poses_6D[n] = optimized_lidar_odometry;
    // poses_edge_points[n] = copy_edge_points;
    // poses_flat_points[n] = copy_flat_points;

    poses_6D.push_back(optimized_lidar_odometry);
    poses_edge_points.push_back(copy_edge_points);
    poses_flat_points.push_back(copy_flat_points);

    if (n == 50) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr output_map(new pcl::PointCloud<pcl::PointXYZI>);
      octreeVoxelGrid(map, output_map, 0.01);
      pcl::io::savePCDFileASCII("/ros2_ws/data/map_lo.pcd", *output_map);
    }
  }

  void computeLocalMap() {

    local_map_edge_points->clear();
    local_map_flat_points->clear();

    // boxFilter.setTranslation(optimized_lidar_odometry.block<3, 1>(0, 3));
    // boxFilter.setInputCloud(poses_3D);
    // boxFilter.filter(*poses_3D);

    RCLCPP_INFO(this->get_logger(), "Nbr pose kept : %d", poses_3D->points.size());

    // std::unordered_map<int, Eigen::Matrix4f> new_poses_6D;
    // std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> new_poses_edge_points;
    // std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> new_poses_flat_points;

    int size_poses_3D = (int) poses_3D->points.size();
    for (int i = size_poses_3D - 1; i >= 0; --i) {

      int idx = poses_3D->points[i].label;

      // pcl::PointCloud<pcl::PointXYZI>::Ptr pose_edge_points_transformed(new pcl::PointCloud<pcl::PointXYZI>());
      // pcl::PointCloud<pcl::PointXYZI>::Ptr pose_flat_points_transformed(new pcl::PointCloud<pcl::PointXYZI>());

      // pcl::transformPointCloud(*poses_edge_points[idx], *pose_edge_points_transformed, poses_6D[idx]);
      // pcl::transformPointCloud(*poses_flat_points[idx], *pose_flat_points_transformed, poses_6D[idx]);

      // *local_map_edge_points += *pose_edge_points_transformed;
      // *local_map_flat_points += *pose_flat_points_transformed;

      *local_map_edge_points += *poses_edge_points[idx];
      *local_map_flat_points += *poses_flat_points[idx];

      // new_poses_6D[idx] = poses_6D[idx];
      // new_poses_edge_points[idx] = poses_edge_points[idx];
      // new_poses_flat_points[idx] = poses_flat_points[idx];

    }

    // poses_6D = new_poses_6D;
    // poses_edge_points = new_poses_edge_points;
    // poses_flat_points = new_poses_flat_points;

    voxel_grid_edge.setInputCloud(local_map_edge_points);
    voxel_grid_edge.filter(*local_map_edge_points);

    voxel_grid_flat.setInputCloud(local_map_flat_points);
    voxel_grid_flat.filter(*local_map_flat_points);

    kdtree_local_map_edge_points->setInputCloud(local_map_edge_points);
    kdtree_local_map_flat_points->setInputCloud(local_map_flat_points);

  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometry>());
  rclcpp::shutdown();
  return 0;
}