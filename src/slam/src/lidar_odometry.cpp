#include <deque>
#include "optimization.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;

class LidarOdometry : public rclcpp::Node {

public:

  LidarOdometry() : Node("lidar_odometry") {

    n = 0;

    new_key_frame_saved = false;

    this->declare_parameter("flat_leaf_size", 0.2f);
    flat_leaf_size = this->get_parameter("flat_leaf_size").get_parameter_value().get<float>();

    this->declare_parameter("edge_leaf_size", 0.1f);
    edge_leaf_size = this->get_parameter("edge_leaf_size").get_parameter_value().get<float>();

    this->declare_parameter("rad_new_key_frame", 0.17f);
    rad_new_key_frame = this->get_parameter("rad_new_key_frame").get_parameter_value().get<float>();

    this->declare_parameter("dist_new_key_frame", 0.2f);
    dist_new_key_frame = this->get_parameter("dist_new_key_frame").get_parameter_value().get<float>();

    this->declare_parameter("local_map_size", 50);
    local_map_size = this->get_parameter("local_map_size").get_parameter_value().get<int>();

    callback_group_cloud = this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions cloud_options = rclcpp::SubscriptionOptions();
    cloud_options.callback_group = callback_group_cloud;

    subscription_cloud_ = this->create_subscription<slam::msg::Cloud>(
      "slam/features", 10, std::bind(&LidarOdometry::cloudHandler, this, std::placeholders::_1), cloud_options
    );

    publisher_lidar_odometry = this->create_publisher<slam::msg::Cloud>("slam/lidar_odometry", 10);

    allocateMemory();

  }

private:

  int n;

  bool new_key_frame_saved;

  // Parameters

  float edge_leaf_size;
  float flat_leaf_size;

  float rad_new_key_frame;
  float dist_new_key_frame;

  int local_map_size;

  // Map

  pcl::PointCloud<pcl::PointXYZI>::Ptr map;

  // Input
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_all_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_flat_points;

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_edge_points_ds;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_flat_points_ds;

  std_msgs::msg::Header input_header;

  Eigen::Matrix4f input_robot_odometry;

  slam::msg::Cloud input_cloud_msg;

  // Optimization

  Eigen::Matrix4f optimized_pose_6D;
  Eigen::Matrix4f estimated_displacement;

  Eigen::Matrix4f last_optimized_pose;

  Eigen::Matrix4f last_robot_odometry;

  // VoxelGrids
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_edge;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat;

  // LocalMap
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> deque_local_map_edge;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> deque_local_map_flat;

  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_flat_points;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_edge_points;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_flat_points;

  rclcpp::Publisher<slam::msg::Cloud>::SharedPtr publisher_lidar_odometry;

  rclcpp::Subscription<slam::msg::Cloud>::SharedPtr subscription_cloud_;
  rclcpp::CallbackGroup::SharedPtr callback_group_cloud;

  void allocateMemory() {
    map.reset(new pcl::PointCloud<pcl::PointXYZI>());

    input_all_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    input_edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    input_flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

    input_edge_points_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());
    input_flat_points_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());

    input_robot_odometry = Eigen::Matrix4f::Identity();

    optimized_pose_6D = Eigen::Matrix4f::Identity();

    estimated_displacement = Eigen::Matrix4f::Identity();

    last_optimized_pose = Eigen::Matrix4f::Identity();

    last_robot_odometry = Eigen::Matrix4f::Identity();

    voxel_grid_flat.setLeafSize(flat_leaf_size, flat_leaf_size, flat_leaf_size);
    voxel_grid_edge.setLeafSize(edge_leaf_size, edge_leaf_size, edge_leaf_size);

    local_map_edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
    local_map_flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>);

    kdtree_local_map_edge_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_local_map_flat_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
  }

  void cloudHandler(const slam::msg::Cloud::SharedPtr cloud_msg) {
    RCLCPP_INFO(this->get_logger(), "Point cloud received : %d", n);

    // if (n > 1000) {
    //   return;
    // }

    // if (n == 1000) {
    //   pcl::PointCloud<pcl::PointXYZI>::Ptr output_map(new pcl::PointCloud<pcl::PointXYZI>);
    //   octreeVoxelGrid(map, output_map, 0.01);
    //   pcl::io::savePCDFileASCII(std::string("data/lidar_odometry.pcd"), *output_map);
    // }

    // Save input
    pcl::fromROSMsg(cloud_msg->cloud_deskewed, *input_all_points);
    pcl::fromROSMsg(cloud_msg->cloud_edge, *input_edge_points);
    pcl::fromROSMsg(cloud_msg->cloud_flat, *input_flat_points);

    input_cloud_msg = *cloud_msg;

    input_header = cloud_msg->header;

    Eigen::Quaternionf rot(cloud_msg->initial_guess_rot_w, cloud_msg->initial_guess_rot_x, cloud_msg->initial_guess_rot_y, cloud_msg->initial_guess_rot_z);
    Eigen::Vector3f trans(cloud_msg->initial_guess_x, cloud_msg->initial_guess_y, cloud_msg->initial_guess_z);

    input_robot_odometry = Eigen::Matrix4f::Identity();
    input_robot_odometry.block<3, 1>(0, 3) = trans;
    input_robot_odometry.block<3, 3>(0, 0) = rot.toRotationMatrix();

    // Down sample input clouds
    voxel_grid_edge.setInputCloud(input_edge_points);
    voxel_grid_edge.filter(*input_edge_points_ds);

    voxel_grid_flat.setInputCloud(input_flat_points);
    voxel_grid_flat.filter(*input_flat_points_ds);

    computeLocalMap();

    optimizeFrame();

    saveKeyFrame();

    ++n;
  }

  void computeLocalMap() {

    if (!new_key_frame_saved) {
      return;
    }

    if (deque_local_map_edge.size() == 0) {
      return;
    }

    local_map_edge_points->clear();
    local_map_flat_points->clear();

    for (int i = 0; i < deque_local_map_edge.size(); ++i) {
      *local_map_edge_points += *deque_local_map_edge[i];
      *local_map_flat_points += *deque_local_map_flat[i];
    }

    voxel_grid_edge.setInputCloud(local_map_edge_points);
    voxel_grid_edge.filter(*local_map_edge_points);

    voxel_grid_flat.setInputCloud(local_map_flat_points);
    voxel_grid_flat.filter(*local_map_flat_points);

    kdtree_local_map_edge_points->setInputCloud(local_map_edge_points);
    kdtree_local_map_flat_points->setInputCloud(local_map_flat_points);

  }

  void optimizeFrame() {

    if (deque_local_map_edge.empty()) {
      return;
    } 

    Eigen::Matrix4f estimated_pose_6D = (last_optimized_pose * estimated_displacement) * getDifferenceTransformation(last_robot_odometry, input_robot_odometry);

    optimized_pose_6D = optimize(
      estimated_pose_6D,
      input_edge_points_ds,
      input_flat_points_ds,
      local_map_edge_points,
      local_map_flat_points,
      kdtree_local_map_edge_points,
      kdtree_local_map_flat_points,
      0.1
    );

    estimated_displacement = getDifferenceTransformation(last_optimized_pose, optimized_pose_6D);

    Eigen::Quaternionf quat(optimized_pose_6D.block<3, 3>(0, 0));

    input_cloud_msg.initial_guess_x = optimized_pose_6D(0, 3);
    input_cloud_msg.initial_guess_y = optimized_pose_6D(1, 3);
    input_cloud_msg.initial_guess_z = optimized_pose_6D(2, 3);
    input_cloud_msg.initial_guess_rot_w = quat.w();
    input_cloud_msg.initial_guess_rot_x = quat.x();
    input_cloud_msg.initial_guess_rot_y = quat.y();
    input_cloud_msg.initial_guess_rot_z = quat.z();

    publisher_lidar_odometry->publish(input_cloud_msg);

  }

  void saveKeyFrame() {

    last_robot_odometry = input_robot_odometry;

    Eigen::Affine3f affine_displacement;
    affine_displacement.matrix() = estimated_displacement;

    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(affine_displacement, x, y, z, roll, pitch, yaw);

    if (!deque_local_map_edge.empty() && std::abs(yaw) < rad_new_key_frame && std::abs(pitch) < rad_new_key_frame && std::abs(roll) < rad_new_key_frame && x * x + y * y + z * z < dist_new_key_frame * dist_new_key_frame) { // Not enough displacement or rotation
      return;
    }

    RCLCPP_INFO(this->get_logger(), "\033[1;32mSave Key Frame\033[0m");

    new_key_frame_saved = true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_edge_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_flat_points(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::transformPointCloud(*input_edge_points, *transformed_key_frame_edge_points, optimized_pose_6D);
    pcl::transformPointCloud(*input_flat_points, *transformed_key_frame_flat_points, optimized_pose_6D); 

    // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_all_points(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::transformPointCloud(*input_all_points, *transformed_all_points, optimized_pose_6D);   

    // *map += *transformed_all_points;

    deque_local_map_edge.push_back(transformed_key_frame_edge_points);
    deque_local_map_flat.push_back(transformed_key_frame_flat_points);

    if (deque_local_map_edge.size() > local_map_size) {
      deque_local_map_edge.pop_front();
      deque_local_map_flat.pop_front();
    }

    last_optimized_pose = optimized_pose_6D;

    estimated_displacement = Eigen::Matrix4f::Identity();
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500));

  auto LO = std::make_shared<LidarOdometry>();
  exec.add_node(LO);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Lidar Odometry Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();

  return 0;
}