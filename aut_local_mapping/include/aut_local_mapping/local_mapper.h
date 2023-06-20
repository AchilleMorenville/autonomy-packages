#ifndef AUT_LOCAL_MAPPING_LOCAL_MAPPER_H_
#define AUT_LOCAL_MAPPING_LOCAL_MAPPER_H_

#include <vector>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include "aut_local_mapping/voxel_grid.h"

#include "aut_msgs/msg/point_cloud_with_pose.hpp"

class Timer {
public:
  Timer() {
    start_time_point = std::chrono::high_resolution_clock::now();
  }
  ~Timer() {
    Stop();
  }

  void Stop() {

    auto end_time_point = std::chrono::high_resolution_clock::now();

    auto start = std::chrono::time_point_cast<std::chrono::milliseconds>(start_time_point).time_since_epoch().count();
    auto end = std::chrono::time_point_cast<std::chrono::milliseconds>(end_time_point).time_since_epoch().count();

    auto duration = end - start;

    std::cout << duration << " ms\n";
  }
private:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_point;
};

namespace aut_local_mapping {

class LocalMapper : public rclcpp::Node {

 public:
  explicit LocalMapper(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());  // TODO: See if we need to put "= default" or "= delete"

 private:

  int n;

  void PointCloudWithPoseCallBack(const aut_msgs::msg::PointCloudWithPose::SharedPtr point_cloud_with_pose_msg);

  void PublishMarkerArray(std::vector<Eigen::Matrix4f>& occupied);
  // VoxelGrid

  VoxelGrid grid;

  // Subscription
  rclcpp::Subscription<aut_msgs::msg::PointCloudWithPose>::SharedPtr point_cloud_with_pose_subscription_;
  
  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_point_cloud_with_pose_;

  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

};

}  // namespace aut_local_mapping

#endif  // AUT_LOCAL_MAPPING_LOCAL_MAPPER_H_