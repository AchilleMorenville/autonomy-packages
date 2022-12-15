#include "utils.hpp"
#include "optimization.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

using namespace std::chrono_literals;

class GraphOptimization : public rclcpp::Node {

public:

  GraphOptimization() : Node("feature_extraction") {

    n = 0;

    // Loop
    loop_is_closed = false;

    process_stopped = true;

    // Graph
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    gtsam::Vector prior_Vector6(6);
    prior_Vector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;

    gtsam::Vector odometry_Vector6(6);
    odometry_Vector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    // odometry_Vector6 << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2;

    gtsam::Vector robust_Vector6(6);
    robust_Vector6 << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2;

    robust_noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
        gtsam::noiseModel::Diagonal::Variances(robust_Vector6)
    );

    prior_noise = gtsam::noiseModel::Diagonal::Variances(prior_Vector6);
    odometry_noise = gtsam::noiseModel::Diagonal::Variances(odometry_Vector6);

    this->declare_parameter("flat_leaf_size", 0.2f);
    flat_leaf_size = this->get_parameter("flat_leaf_size").get_parameter_value().get<float>();

    this->declare_parameter("edge_leaf_size", 0.1f);
    edge_leaf_size = this->get_parameter("edge_leaf_size").get_parameter_value().get<float>();

    this->declare_parameter("ICP_leaf_size", 0.2f);
    ICP_leaf_size = this->get_parameter("ICP_leaf_size").get_parameter_value().get<float>();

    this->declare_parameter("local_map_size", 50);
    local_map_size = this->get_parameter("local_map_size").get_parameter_value().get<int>();

    this->declare_parameter("rad_new_key_frame", 0.17f);
    rad_new_key_frame = this->get_parameter("rad_new_key_frame").get_parameter_value().get<float>();

    this->declare_parameter("dist_new_key_frame", 0.2f);
    dist_new_key_frame = this->get_parameter("dist_new_key_frame").get_parameter_value().get<float>();

    this->declare_parameter("loop_search_dist", 2.0f);
    loop_search_dist = this->get_parameter("loop_search_dist").get_parameter_value().get<float>();

    this->declare_parameter("loop_closure_frequency", 1.0f);
    loop_closure_frequency = this->get_parameter("loop_closure_frequency").get_parameter_value().get<float>();

    subscription_ = this->create_subscription<slam::msg::Cloud>(
      "slam/cloud", 10, std::bind(&GraphOptimization::cloudHandler, this, std::placeholders::_1)
    );

    srv_SaveMap = this->create_service<autonomous_interfaces::srv::SlamSaveMap>("slam/save_map", std::bind(&GraphOptimization::saveMap, this, std::placeholders::_1, std::placeholders::_2));
    srv_Start = this->create_service<autonomous_interfaces::srv::SlamStart>("slam/start", std::bind(&GraphOptimization::startProcess, this, std::placeholders::_1, std::placeholders::_2));
    srv_Stop = this->create_service<autonomous_interfaces::srv::SlamStop>("slam/stop", std::bind(&GraphOptimization::stopProcess, this, std::placeholders::_1, std::placeholders::_2));
    srv_Reset = this->create_service<autonomous_interfaces::srv::SlamReset>("slam/reset", std::bind(&GraphOptimization::resetProcess, this, std::placeholders::_1, std::placeholders::_2));

    allocateMemory();
  }

  void loopClosureThread() {
    rclcpp::Rate rate(loop_closure_frequency);
    while (rclcpp::ok()) {
      rate.sleep();

      if (process_stopped) {
        continue;
      }

      performLoopClosure();
    }
  }

private:

  int n;

  bool process_stopped;

  // Parameters

  float edge_leaf_size;
  float flat_leaf_size;

  int local_map_size;

  float rad_new_key_frame;
  float dist_new_key_frame;

  float loop_search_dist;

  float ICP_leaf_size;

  float loop_closure_frequency;

  // Input
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_all_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_flat_points;

  std_msgs::msg::Header input_header;

  Eigen::Matrix4f input_robot_odometry;

  // VoxelGrids
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_edge;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat;

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_ICP;

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_edge_loop_closure;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_flat_loop_closure;

  // Key Frames
  std::vector<Eigen::Matrix4f> poses_6D;
  pcl::PointCloud<pcl::PointXYZL>::Ptr poses_3D;

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_frames_edge_points;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_frames_flat_points;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_frames_all_points;

  pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kdtree_poses_3D;

  // Local Map
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_edge_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_flat_points;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_edge_points;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_local_map_flat_points;

  // Optimization
  Eigen::Matrix4f estimated_displacement;
  Eigen::Matrix4f last_robot_odometry;
  Eigen::Matrix4f optimized_pose_6D;

  // Graph
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_estimate;
  gtsam::Values optimized_estimate;
  gtsam::ISAM2 *isam;
  gtsam::Values isam_current_estimate;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise;
  gtsam::noiseModel::Diagonal::shared_ptr constraint_noise;
  gtsam::noiseModel::Base::shared_ptr robust_noise;

  // Loop
  bool loop_is_closed;

  std::vector<gtsam::BetweenFactor<gtsam::Pose3>> loop_factors;

  // Mutex
  std::mutex mtx;

  // Subscription
  rclcpp::Subscription<slam::msg::Cloud>::SharedPtr subscription_;

  rclcpp::Service<autonomous_interfaces::srv::SlamSaveMap>::SharedPtr srv_SaveMap;
  rclcpp::Service<autonomous_interfaces::srv::SlamStart>::SharedPtr srv_Start;
  rclcpp::Service<autonomous_interfaces::srv::SlamStop>::SharedPtr srv_Stop;
  rclcpp::Service<autonomous_interfaces::srv::SlamReset>::SharedPtr srv_Reset;

  void allocateMemory() {

    // Input
    input_all_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    input_edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
    input_flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>());

    input_robot_odometry = Eigen::Matrix4f::Identity();

    // VoxelGrids
    voxel_grid_flat.setLeafSize(flat_leaf_size, flat_leaf_size, flat_leaf_size);
    voxel_grid_edge.setLeafSize(edge_leaf_size, edge_leaf_size, edge_leaf_size);

    voxel_grid_ICP.setLeafSize(ICP_leaf_size, ICP_leaf_size, ICP_leaf_size);

    voxel_grid_flat_loop_closure.setLeafSize(flat_leaf_size, flat_leaf_size, flat_leaf_size);
    voxel_grid_edge_loop_closure.setLeafSize(edge_leaf_size, edge_leaf_size, edge_leaf_size);

    // Key Frames
    poses_3D.reset(new pcl::PointCloud<pcl::PointXYZL>());

    kdtree_poses_3D.reset(new pcl::KdTreeFLANN<pcl::PointXYZL>);

    // Local Map
    local_map_edge_points.reset(new pcl::PointCloud<pcl::PointXYZI>);
    local_map_flat_points.reset(new pcl::PointCloud<pcl::PointXYZI>);

    kdtree_local_map_edge_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    kdtree_local_map_flat_points.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);

    // Optimization
    estimated_displacement = Eigen::Matrix4f::Identity();
    last_robot_odometry = Eigen::Matrix4f::Identity();
    optimized_pose_6D = Eigen::Matrix4f::Identity();
  }

  void cloudHandler(const slam::msg::Cloud::SharedPtr cloud_msg) {

    if (process_stopped) {
      return;
    }

    // Save input

    pcl::fromROSMsg(cloud_msg->cloud_deskewed, *input_all_points);
    pcl::fromROSMsg(cloud_msg->cloud_edge, *input_edge_points);
    pcl::fromROSMsg(cloud_msg->cloud_flat, *input_flat_points);

    input_header = cloud_msg->header;

    Eigen::Quaternionf rot(cloud_msg->initial_guess_rot_w, cloud_msg->initial_guess_rot_x, cloud_msg->initial_guess_rot_y, cloud_msg->initial_guess_rot_z);
    Eigen::Vector3f trans(cloud_msg->initial_guess_x, cloud_msg->initial_guess_y, cloud_msg->initial_guess_z);

    input_robot_odometry = Eigen::Matrix4f::Identity();
    input_robot_odometry.block<3, 1>(0, 3) = trans;
    input_robot_odometry.block<3, 3>(0, 0) = rot.toRotationMatrix();

    // Down sample input clouds
    voxel_grid_edge.setInputCloud(input_edge_points);
    voxel_grid_edge.filter(*input_edge_points);

    voxel_grid_flat.setInputCloud(input_flat_points);
    voxel_grid_flat.filter(*input_flat_points);

    {
      std::lock_guard<std::mutex> lock(mtx);

      extractLocalMap();

      optimizeFrame();

      saveKeyFrameAndFactor();

      updatePoses();
    }

    ++n;
  }

  void extractLocalMap() {

    if (poses_3D->points.empty() == true) { // If there is no keyframe to extract then skip
      return;
    }

    // Reset local map cloud
    local_map_edge_points->clear();
    local_map_flat_points->clear();

    // Exctract local map
    for (int i = poses_3D->points.size() - 1; i >= 0; --i) {

      if (local_map_size < ((int) poses_3D->points.size()) - i) { // Extract the correct number of key frames
        break;
      }

      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_edge_points(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_flat_points(new pcl::PointCloud<pcl::PointXYZI>);

      pcl::transformPointCloud(*key_frames_edge_points[i], *transformed_key_frame_edge_points, poses_6D[i]); // TODO: Cache transformation
      pcl::transformPointCloud(*key_frames_flat_points[i], *transformed_key_frame_flat_points, poses_6D[i]);

      *local_map_edge_points += *transformed_key_frame_edge_points;
      *local_map_flat_points += *transformed_key_frame_flat_points;
    }

    voxel_grid_edge.setInputCloud(local_map_edge_points);
    voxel_grid_edge.filter(*local_map_edge_points);

    voxel_grid_flat.setInputCloud(local_map_flat_points);
    voxel_grid_flat.filter(*local_map_flat_points);

    kdtree_local_map_edge_points->setInputCloud(local_map_edge_points);
    kdtree_local_map_flat_points->setInputCloud(local_map_flat_points);
  }

  void optimizeFrame() {

    if (poses_3D->points.empty() == true) { // If there is no optimization to be done
      return;
    }

    // Estimated pose using the estimated displacement and odometry
    Eigen::Matrix4f estimated_pose_6D = (poses_6D[poses_6D.size() - 1] * estimated_displacement) * getDifferenceTransformation(last_robot_odometry, input_robot_odometry);

    optimized_pose_6D = optimize(
      estimated_pose_6D,
      input_edge_points,
      input_flat_points,
      local_map_edge_points,
      local_map_flat_points,
      kdtree_local_map_edge_points,
      kdtree_local_map_flat_points,
      0.20
    );

    estimated_displacement = getDifferenceTransformation(poses_6D[poses_6D.size() - 1], optimized_pose_6D);
  }

  void saveKeyFrameAndFactor() {

    last_robot_odometry = input_robot_odometry;

    if (poses_3D->points.empty() == true) { // There is no key frame saved

      graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(optimized_pose_6D.cast<double>()), prior_noise));

      initial_estimate.insert(0, gtsam::Pose3(optimized_pose_6D.cast<double>()));

      isam->update(graph, initial_estimate);
      isam->update();

      graph.resize(0);
      initial_estimate.clear();

      estimated_displacement = Eigen::Matrix4f::Identity();
      poses_6D.push_back(optimized_pose_6D);

      pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_edge_points(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_flat_points(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_all_points(new pcl::PointCloud<pcl::PointXYZI>());

      pcl::copyPointCloud(*input_edge_points,  *copy_input_edge_points);
      pcl::copyPointCloud(*input_flat_points,  *copy_input_flat_points);
      pcl::copyPointCloud(*input_all_points,  *copy_input_all_points);

      key_frames_edge_points.push_back(copy_input_edge_points);
      key_frames_flat_points.push_back(copy_input_flat_points);
      key_frames_all_points.push_back(copy_input_all_points);

      pcl::PointXYZL point;
      point.x = optimized_pose_6D(0, 3);
      point.y = optimized_pose_6D(1, 3);
      point.z = optimized_pose_6D(2, 3);
      point.label = poses_3D->points.size();

      poses_3D->push_back(point);

    } else {

      Eigen::Matrix4f displacement = getDifferenceTransformation(poses_6D[poses_6D.size() - 1], optimized_pose_6D);

      Eigen::Affine3f affine_displacement;
      affine_displacement.matrix() = displacement;

      float x, y, z, roll, pitch, yaw;
      pcl::getTranslationAndEulerAngles(affine_displacement, x, y, z, roll, pitch, yaw);

      if (std::abs(yaw) < rad_new_key_frame && std::abs(pitch) < rad_new_key_frame && std::abs(roll) < rad_new_key_frame && x * x + y * y + z * z < dist_new_key_frame * dist_new_key_frame) { // Not enough displacement or rotation
        return;
      }

      gtsam::Pose3 pose_from = gtsam::Pose3(poses_6D[poses_6D.size() - 1].cast<double>());

      gtsam::Pose3 pose_to = gtsam::Pose3(optimized_pose_6D.cast<double>());

      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(poses_6D.size() - 1, poses_6D.size(), pose_from.between(pose_to), odometry_noise));

      if (loop_factors.size() > 0) {
        for (int i = 0; i < (int) loop_factors.size(); i++) {
          graph.add(loop_factors[i]);
        }

        loop_factors.clear();

        loop_is_closed = true;
      }

      initial_estimate.insert(poses_6D.size(), gtsam::Pose3(optimized_pose_6D.cast<double>()));

      isam->update(graph, initial_estimate);
      isam->update();

      if (loop_is_closed == true || true) {
          isam->update();
          isam->update();
          isam->update();
          isam->update();
          isam->update();
      }

      graph.resize(0);
      initial_estimate.clear();

      estimated_displacement = Eigen::Matrix4f::Identity();
      poses_6D.push_back(optimized_pose_6D);

      pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_edge_points(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_flat_points(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr copy_input_all_points(new pcl::PointCloud<pcl::PointXYZI>());

      pcl::copyPointCloud(*input_edge_points,  *copy_input_edge_points);
      pcl::copyPointCloud(*input_flat_points,  *copy_input_flat_points);
      pcl::copyPointCloud(*input_all_points,  *copy_input_all_points);

      key_frames_edge_points.push_back(copy_input_edge_points);
      key_frames_flat_points.push_back(copy_input_flat_points);
      key_frames_all_points.push_back(copy_input_all_points);

      pcl::PointXYZL point;
      point.x = optimized_pose_6D(0, 3);
      point.y = optimized_pose_6D(1, 3);
      point.z = optimized_pose_6D(2, 3);
      point.label = poses_3D->points.size();

      poses_3D->push_back(point);
    }
  }

  void updatePoses() {
    if (loop_is_closed) {
      isam_current_estimate = isam->calculateEstimate();
      for (int i = 0; i < (int) isam_current_estimate.size(); i++) {

        Eigen::Matrix4f estimate = isam_current_estimate.at<gtsam::Pose3>(i).matrix().cast<float>();

        poses_3D->points[i].x = estimate(0, 3);
        poses_3D->points[i].y = estimate(1, 3);
        poses_3D->points[i].z = estimate(2, 3);

        poses_6D[i] = estimate;
      }

      loop_is_closed = false;
    }
  }

  void performLoopClosure() {
    if (poses_3D->points.empty() == true || loop_is_closed) {
      return;
    }

    pcl::PointCloud<pcl::PointXYZL>::Ptr copy_poses_3D(new pcl::PointCloud<pcl::PointXYZL>);

    mtx.lock();
    *copy_poses_3D += *poses_3D;
    // std::vector<Eigen::Matrix4f> copy_poses_6D(poses_6D);
    mtx.unlock();

    // Select a pose for loop closure
    std::vector<int> poses_search_idx;
    std::vector<float> poses_search_sq_dist;

    kdtree_poses_3D->setInputCloud(copy_poses_3D);
    kdtree_poses_3D->radiusSearch(copy_poses_3D->points[copy_poses_3D->points.size() - 1], loop_search_dist, poses_search_idx, poses_search_sq_dist, 0);

    int selected_pose_idx = -1;
    int new_pose_idx = copy_poses_3D->points.size() - 1;

    int oldest_id = std::numeric_limits<int>::max();

    for (int i = 0; i < (int) poses_search_idx.size(); i++) {
      if (new_pose_idx - poses_search_idx[i] > local_map_size) {
        selected_pose_idx = poses_search_idx[i];
        oldest_id = poses_search_idx[i];
        break;
      }
    }

    if (selected_pose_idx < 0) { // Didn't find loop closure
      return;
    }

    mtx.lock();

    Eigen::Matrix4f selected_pose = poses_6D[selected_pose_idx];
    Eigen::Matrix4f new_pose = poses_6D[new_pose_idx];

    // Construct new pose pcd and select pose pcd
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pose_edge_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pose_flat_points(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::copyPointCloud(*key_frames_edge_points[new_pose_idx], *new_pose_edge_points);
    pcl::copyPointCloud(*key_frames_flat_points[new_pose_idx], *new_pose_flat_points);
    mtx.unlock();

    pcl::PointCloud<pcl::PointXYZI>::Ptr selected_pose_edge_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr selected_pose_flat_points(new pcl::PointCloud<pcl::PointXYZI>);

    for (int i = - local_map_size / 2; i < local_map_size / 2; i++) { // Take the 100 frames around the closest pose
      if (selected_pose_idx + i >= 0 && selected_pose_idx + i < new_pose_idx + 1){

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_selected_pose_edge_points(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_selected_pose_flat_points(new pcl::PointCloud<pcl::PointXYZI>);

        // mtx.lock();
        pcl::transformPointCloud(*key_frames_edge_points[selected_pose_idx + i], *transformed_selected_pose_edge_points, poses_6D[selected_pose_idx + i]);
        pcl::transformPointCloud(*key_frames_flat_points[selected_pose_idx + i], *transformed_selected_pose_flat_points, poses_6D[selected_pose_idx + i]);
        // mtx.unlock();

        *selected_pose_edge_points += *transformed_selected_pose_edge_points;
        *selected_pose_flat_points += *transformed_selected_pose_flat_points;
      }
    }

    mtx.unlock();

    // First execute ICP to be sure it is a loop
    pcl::PointCloud<pcl::PointXYZI>::Ptr selected_pose_points(new pcl::PointCloud<pcl::PointXYZI>);
    *selected_pose_points += *selected_pose_edge_points;
    *selected_pose_points += *selected_pose_flat_points;

    voxel_grid_ICP.setInputCloud(selected_pose_points);
    voxel_grid_ICP.filter(*selected_pose_points);

    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pose_points(new pcl::PointCloud<pcl::PointXYZI>);
    *new_pose_points += *new_pose_edge_points;
    *new_pose_points += *new_pose_flat_points;

    voxel_grid_ICP.setInputCloud(new_pose_points);
    voxel_grid_ICP.filter(*new_pose_points);

    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
    icp.setMaxCorrespondenceDistance(0.3);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr new_pose_points_transformed(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*new_pose_points, *new_pose_points_transformed, new_pose);

    icp.setInputSource(new_pose_points_transformed);
    icp.setInputTarget(selected_pose_points);
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>());
    icp.align(*result);

    if (icp.hasConverged() && icp.getFitnessScore(0.3) < 0.1) { // ICP found a potential loop

      // Refine estimation using LOAM
      voxel_grid_edge_loop_closure.setInputCloud(selected_pose_edge_points);
      voxel_grid_edge_loop_closure.filter(*selected_pose_edge_points);

      voxel_grid_flat_loop_closure.setInputCloud(selected_pose_flat_points);
      voxel_grid_flat_loop_closure.filter(*selected_pose_flat_points);

      pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_selected_pose_edge_points(new pcl::KdTreeFLANN<pcl::PointXYZI>);
      pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_selected_pose_flat_points(new pcl::KdTreeFLANN<pcl::PointXYZI>);

      kdtree_selected_pose_edge_points->setInputCloud(selected_pose_edge_points);
      kdtree_selected_pose_flat_points->setInputCloud(selected_pose_flat_points);

      Eigen::Matrix4f best_estimation = optimize(
        icp.getFinalTransformation() * new_pose,
        new_pose_edge_points,
        new_pose_flat_points,
        selected_pose_edge_points,
        selected_pose_flat_points,
        kdtree_selected_pose_edge_points,
        kdtree_selected_pose_flat_points,
        0.1
      );

      gtsam::Pose3 pose_from = gtsam::Pose3(
        best_estimation.cast<double>()
      );

      gtsam::Pose3 pose_to = gtsam::Pose3(
        selected_pose.cast<double>()
      );


      gtsam::Vector Vector6(6);
      float noiseScore = icp.getFitnessScore();
      Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
      gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);

      mtx.lock();
      loop_factors.push_back(gtsam::BetweenFactor<gtsam::Pose3>(new_pose_idx, selected_pose_idx, pose_from.between(pose_to), odometry_noise));
      loop_is_closed = true;
      mtx.unlock();

    }
  }

  void saveMap(const std::shared_ptr<autonomous_interfaces::srv::SlamSaveMap::Request> request, std::shared_ptr<autonomous_interfaces::srv::SlamSaveMap::Response> response) {
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);

    mtx.lock();
    int poses_3D_size = (int) poses_3D->points.size();
    mtx.unlock();
    
    for (int i = 0; i < poses_3D_size; ++i) {

      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_key_frame_all_points(new pcl::PointCloud<pcl::PointXYZI>);

      mtx.lock();
      pcl::transformPointCloud(*key_frames_all_points[i], *transformed_key_frame_all_points, poses_6D[i]);
      mtx.unlock();

      *map += *transformed_key_frame_all_points;
    }

    float resolution = request->resolution;

    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_map;
    voxel_grid_map.setLeafSize(resolution, resolution, resolution);
    voxel_grid_map.setInputCloud(map);
    voxel_grid_map.filter(*map);

    pcl::io::savePCDFileASCII(request->destination, *map);

    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Map saved");

    return;
  }

  void stopProcess(const std::shared_ptr<autonomous_interfaces::srv::SlamStop::Request> request, std::shared_ptr<autonomous_interfaces::srv::SlamStop::Response> response) {
    process_stopped = true;

    rclcpp::Rate rate(0.5);
    rate.sleep();
    
    // rclcpp::sleep_for(std::chrono::nanoseconds(2s).count());

    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Process stopped");

    return;
  }

  void startProcess(const std::shared_ptr<autonomous_interfaces::srv::SlamStart::Request> request, std::shared_ptr<autonomous_interfaces::srv::SlamStart::Response> response) {
    process_stopped = false;
    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Process started");

    return;
  }

  void resetProcess(const std::shared_ptr<autonomous_interfaces::srv::SlamReset::Request> request, std::shared_ptr<autonomous_interfaces::srv::SlamReset::Response> response) {
    if (!process_stopped) {
      response->success = false;
      return;
    }

    mtx.lock();

    n = 0;

    poses_3D->clear();

    poses_6D.clear();

    key_frames_edge_points.clear();
    key_frames_flat_points.clear();
    key_frames_all_points.clear();

    estimated_displacement = Eigen::Matrix4f::Identity();
    last_robot_odometry = Eigen::Matrix4f::Identity();
    optimized_pose_6D = Eigen::Matrix4f::Identity();

    loop_factors.clear();

    graph.resize(0);
    initial_estimate.clear();

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

    mtx.unlock();

    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Process reset");

    return;
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);
  rclcpp::executors::SingleThreadedExecutor exec;

  auto GO = std::make_shared<GraphOptimization>();
  exec.add_node(GO);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Graph Optimization Started.\033[0m");

  std::thread loopThread(&GraphOptimization::loopClosureThread, GO);

  exec.spin();

  rclcpp::shutdown();

  loopThread.join();

  return 0;
}