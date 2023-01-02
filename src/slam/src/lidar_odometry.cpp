#include "utils.hpp"
#include "optimization.hpp"

class LidarOdometry : public rclcpp::Node {

public:

  LidarOdometry() : Node("lidar_odometry") {

    n = 0;

    k = 0;

    this->declare_parameter("flat_leaf_size", 0.2f);
    flat_leaf_size = this->get_parameter("flat_leaf_size").get_parameter_value().get<float>();

    this->declare_parameter("edge_leaf_size", 0.1f);
    edge_leaf_size = this->get_parameter("edge_leaf_size").get_parameter_value().get<float>();

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    cloud_subscription_ = this->create_subscription<slam::msg::Cloud>(
      "slam/feature_extraction/cloud", 10, std::bind(&LidarOdometry::cloudHandler, this, std::placeholders::_1)
    );

    allocateMemory();
  }

private:

  int n;
  int k;

  // Parameters

  float flat_leaf_size;
  float edge_leaf_size;

  // Input

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_all_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_flat_points;

  std_msgs::msg::Header input_header;

  long input_start_time_cloud;
  long input_end_time_cloud;

  float input_start_point_time;
  float input_end_point_time;

  Eigen::Matrix4f input_vision_displacement;

  geometry_msgs::msg::TransformStamped t_start;
  geometry_msgs::msg::TransformStamped t_end;

  Eigen::Matrix4f vision_odometry;

  // Last

  Eigen::Matrix4f last_lidar_odometry;
  Eigen::Matrix4f last_vision_odometry;

  // Optimization

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_edge;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat;

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

  // pcl::CropBox<pcl::PointXYZL> boxFilter;

  // All map

  pcl::PointCloud<pcl::PointXYZI>::Ptr map;

  // Subscription
  rclcpp::Subscription<slam::msg::Cloud>::SharedPtr cloud_subscription_;

  // tf2
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  void allocateMemory() {
    // Input
    input_all_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    input_edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    input_flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

    voxel_grid_edge.setLeafSize(edge_leaf_size, edge_leaf_size, edge_leaf_size);
    voxel_grid_flat.setLeafSize(flat_leaf_size, flat_leaf_size, flat_leaf_size);

    input_vision_displacement = Eigen::Matrix4f::Identity();
    last_lidar_odometry = Eigen::Matrix4f::Identity();
    optimized_lidar_odometry = Eigen::Matrix4f::Identity();
    vision_odometry = Eigen::Matrix4f::Identity();
    last_vision_odometry = Eigen::Matrix4f::Identity();

    local_map_edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    local_map_flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

    kdtree_local_map_edge_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    kdtree_local_map_flat_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

    poses_3D.reset(new pcl::PointCloud<pcl::PointXYZL>());

    map.reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  void cloudHandler(const slam::msg::Cloud::SharedPtr cloud_msg) {

    if (k < 2) {
      ++k;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Recieved : %d", n);

    // Save current input

    pcl::fromROSMsg(cloud_msg->cloud_all, *input_all_points);
    pcl::fromROSMsg(cloud_msg->cloud_edge, *input_edge_points);
    pcl::fromROSMsg(cloud_msg->cloud_flat, *input_flat_points);

    input_header = cloud_msg->header;

    input_start_point_time = cloud_msg->start_point_time;
    input_end_point_time = cloud_msg->end_point_time;

    input_start_time_cloud = rclcpp::Time(input_header.stamp).nanoseconds() + int(input_start_point_time * 1e9);
    input_end_time_cloud = rclcpp::Time(input_header.stamp).nanoseconds() + int(input_end_point_time * 1e9);

    try {

      t_start = tf_buffer_->lookupTransform(
            "vision", "velodyne",
            rclcpp::Time(input_start_time_cloud));

      t_end = tf_buffer_->lookupTransform(
              "vision", "velodyne",
              rclcpp::Time(input_end_time_cloud));

    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(this->get_logger(), "Could not find transform : %s", ex.what());
    }

    Eigen::Quaternionf rot_start(t_start.transform.rotation.w, t_start.transform.rotation.x, t_start.transform.rotation.y, t_start.transform.rotation.z);
    Eigen::Quaternionf rot_end(t_end.transform.rotation.w, t_end.transform.rotation.x, t_end.transform.rotation.y, t_end.transform.rotation.z);

    Eigen::Vector3f trans_start(t_start.transform.translation.x, t_start.transform.translation.y, t_start.transform.translation.z);
    Eigen::Vector3f trans_end(t_end.transform.translation.x, t_end.transform.translation.y, t_end.transform.translation.z);

    Eigen::Affine3f affine_start;
    affine_start.translation() = trans_start;
    affine_start.linear() = rot_start.toRotationMatrix();

    Eigen::Affine3f affine_end;
    affine_end.translation() = trans_end;
    affine_end.linear() = rot_end.toRotationMatrix();

    Eigen::Affine3f affine_diff = affine_start.inverse() * affine_end;

    input_vision_displacement = affine_diff.matrix();

    vision_odometry = affine_start.matrix();

    // deskewCloudToEnd(input_edge_points);
    // deskewCloudToEnd(input_flat_points);
    // deskewCloudToEnd(input_all_points);

    deskewCloudToStart(input_edge_points);
    deskewCloudToStart(input_flat_points);
    deskewCloudToStart(input_all_points);

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

      ++n;
      ++k;

    }

  }

  void deskewCloudToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    int size_cloud = (int) cloud->points.size();
    for (int i = 0; i < size_cloud; ++i) {
      pcl::PointXYZI point = cloud->points[i];

      float alpha = (input_end_point_time - point.intensity) / (input_end_point_time - input_start_point_time);

      Eigen::Vector3f vec_point(point.x, point.y, point.z);
      Eigen::Matrix3f rot_T = input_vision_displacement.block<3, 3>(0, 0).transpose();

      Eigen::Quaternionf q_identity, q_displacement, q_final;
      q_displacement = rot_T;
      q_identity.setIdentity();
      q_final = q_identity.slerp(alpha, q_displacement);

      vec_point = q_final * vec_point + alpha * (- rot_T * input_vision_displacement.block<3, 1>(0, 3));

      cloud->points[i].x = vec_point(0);
      cloud->points[i].y = vec_point(1);
      cloud->points[i].z = vec_point(2);
    }
  }

  void deskewCloudToStart(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
    int size_cloud = (int) cloud->points.size();
    for (int i = 0; i < size_cloud; ++i) {
      pcl::PointXYZI point = cloud->points[i];

      float alpha = (point.intensity - input_start_point_time) / (input_end_point_time - input_start_point_time);

      Eigen::Vector3f vec_point(point.x, point.y, point.z);

      Eigen::Quaternionf q_identity, q_displacement, q_final;
      q_displacement = input_vision_displacement.block<3, 3>(0, 0).transpose();
      q_identity.setIdentity();
      q_final = q_identity.slerp(alpha, q_displacement);

      vec_point = q_final * vec_point + alpha * input_vision_displacement.block<3, 1>(0, 3);

      cloud->points[i].x = vec_point(0);
      cloud->points[i].y = vec_point(1);
      cloud->points[i].z = vec_point(2);
    }
  }

  // void deskewPoint(pcl::PointXYZI & p, Eigen::Matrix4f transformation, float point_time) {

  //   float alpha = (point_time - start_point_time) / (end_point_time - start_point_time);

  //   // Eigen::Affine3f affine_correction;
  //   // affine_correction.translation() = alpha * transformation.block<3, 1>(0, 3);

  //   // Eigen::Quaternionf quat_transformation;
  //   // quat_transformation = transformation.block<3, 3>(0, 0);

  //   // Eigen::Quaternionf quat_identity;
  //   // quat_identity = Eigen::Matrix3f::Identity();
  //   // // Eigen::Quaternionf quat_identity(1, 0, 0, 0);

  //   // affine_correction.linear() = quat_identity.slerp(alpha, quat_transformation).toRotationMatrix();

  //   // Eigen::Vector3f vec_point(p.x, p.y, p.z);
  //   // vec_point = affine_correction * vec_point;

  //   if (alpha < 0 || alpha > 1.001) {
  //     RCLCPP_INFO(this->get_logger(), "Wrong ratio : %f", alpha);
  //   }

  //   Eigen::Vector3f vec_point(p.x, p.y, p.z);

  //   vec_point(0) = p.x - alpha * transformation(0, 3);
  //   vec_point(1) = p.y - alpha * transformation(1, 3);
  //   vec_point(2) = p.z - alpha * transformation(2, 3);

  //   Eigen::Quaternionf q_id, q_es, q_f;
  //   q_es = transformation.block<3, 3>(0, 0);
  //   q_id.setIdentity();
  //   q_f = q_id.slerp(alpha, q_es);

  //   vec_point = q_f.conjugate() * vec_point;

  //   p.x = vec_point(0);
  //   p.y = vec_point(1);
  //   p.z = vec_point(2);
  // }

  // void deskewPointEnd(pcl::PointXYZI & p, Eigen::Matrix4f displacement) {

  //   float alpha = (end_point_time - p.intensity) / (end_point_time - start_point_time);

  //   // Eigen::Affine3f affine_correction;
  //   // affine_correction.translation() = alpha * transformation.block<3, 1>(0, 3);

  //   // Eigen::Quaternionf quat_transformation;
  //   // quat_transformation = transformation.block<3, 3>(0, 0);

  //   // Eigen::Quaternionf quat_identity;
  //   // quat_identity = Eigen::Matrix3f::Identity();
  //   // // Eigen::Quaternionf quat_identity(1, 0, 0, 0);

  //   // affine_correction.linear() = quat_identity.slerp(alpha, quat_transformation).toRotationMatrix();

  //   // Eigen::Vector3f vec_point(p.x, p.y, p.z);
  //   // vec_point = affine_correction * vec_point;

  //   if (alpha < 0 || alpha > 1.001) {
  //     RCLCPP_INFO(this->get_logger(), "Wrong ratio : %f", alpha);
  //   }

  //   Eigen::Vector3f vec_point(p.x, p.y, p.z);
  //   Eigen::Matrix3f rot_T = displacement.block<3, 3>(0, 0).transpose();

  //   Eigen::Quaternionf q_identity, q_displacement, q_final;
  //   q_displacement = rot_T;
  //   q_identity.setIdentity();
  //   q_final = q_identity.slerp(alpha, q_displacement);

  //   vec_point = q_final * vec_point + alpha * (- rot_T * displacement.block<3, 1>(0, 3));

  //   p.x = vec_point(0);
  //   p.y = vec_point(1);
  //   p.z = vec_point(2);
  // }

  void optimizeCurrent() {

    if (poses_3D->points.empty() == true) { // If there is no optimization to be done
      return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points_ds(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr flat_points_ds(new pcl::PointCloud<pcl::PointXYZI>());

    voxel_grid_edge.setInputCloud(input_edge_points);
    voxel_grid_edge.filter(*input_edge_points);

    voxel_grid_flat.setInputCloud(input_flat_points);
    voxel_grid_flat.filter(*input_flat_points);

    RCLCPP_INFO(this->get_logger(), "Nbr flat points : %d", (int) flat_points_ds->points.size());
    RCLCPP_INFO(this->get_logger(), "Nbr edge points : %d", (int) edge_points_ds->points.size());

    // Estimated pose using the estimated displacement and odometry
    Eigen::Matrix4f estimated_lidar_odometry = last_lidar_odometry * getDifferenceTransformation(last_vision_odometry, vision_odometry);

    optimized_lidar_odometry = optimize(
      estimated_lidar_odometry,
      input_edge_points,
      input_flat_points,
      local_map_edge_points,
      local_map_flat_points,
      kdtree_local_map_edge_points,
      kdtree_local_map_flat_points,
      0.20
    );

  }

  void saveCurrent() {
    // displacement_estimation = getDifferenceTransformation(last_lidar_odometry, optimized_lidar_odometry);
    
    last_lidar_odometry = optimized_lidar_odometry;
    last_vision_odometry = vision_odometry;

    pcl::PointXYZL point;
    point.x = optimized_lidar_odometry(0, 3);
    point.y = optimized_lidar_odometry(1, 3);
    point.z = optimized_lidar_odometry(2, 3);
    point.label = n;

    poses_3D->push_back(point);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_all_points_transformed(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::transformPointCloud(*input_all_points, *input_all_points_transformed, optimized_lidar_odometry);

    *map += *input_all_points_transformed;

    // TODO: Final deskewing

    pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_edge_points(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_flat_points(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::copyPointCloud(*input_edge_points,  *copy_input_edge_points);
    pcl::copyPointCloud(*input_flat_points,  *copy_input_flat_points);

    pcl::transformPointCloud(*copy_input_edge_points, *copy_input_edge_points, optimized_lidar_odometry);
    pcl::transformPointCloud(*copy_input_flat_points, *copy_input_flat_points, optimized_lidar_odometry);

    // poses_6D[n] = optimized_lidar_odometry;
    // poses_edge_points[n] = copy_edge_points;
    // poses_flat_points[n] = copy_flat_points;

    poses_6D.push_back(optimized_lidar_odometry);
    poses_edge_points.push_back(copy_input_edge_points);
    poses_flat_points.push_back(copy_input_flat_points);

    if (n == 400) {
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

    RCLCPP_INFO(this->get_logger(), "Nbr pose kept : %d", (int) poses_3D->points.size());

    // std::unordered_map<int, Eigen::Matrix4f> new_poses_6D;
    // std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> new_poses_edge_points;
    // std::unordered_map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> new_poses_flat_points;

    int size_poses_3D = (int) poses_3D->points.size();
    for (int i = size_poses_3D - 1; i >= 0; --i) {

      if (size_poses_3D - i - 1 >= 100) {
        break;
      }

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