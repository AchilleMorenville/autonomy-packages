#ifndef AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_
#define AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_

#include <mutex>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <aut_msgs/msg/local_grid.hpp>
#include <aut_msgs/msg/nav.hpp>
#include <aut_msgs/msg/nav_modif.hpp>
#include <aut_msgs/msg/nav_command.hpp>

#include "aut_common/transformer.h"
#include <aut_local_planner/local_grid.h>

namespace aut_local_planner {

class LocalPlanner : public rclcpp::Node {

 public:
  explicit LocalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg);
  void NavCallBack(const aut_msgs::msg::Nav::SharedPtr nav_msg);

  // State
  LocalGrid local_grid_;

  // Publisher
  rclcpp::Publisher<aut_msgs::msg::NavCommand>::SharedPtr nav_command_publisher_;
  rclcpp::Publisher<aut_msgs::msg::NavModif>::SharedPtr nav_modif_publisher_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_command_publisher_;

  // Subscription
  rclcpp::Subscription<aut_msgs::msg::LocalGrid>::SharedPtr local_grid_subscription_;
  rclcpp::Subscription<aut_msgs::msg::Nav>::SharedPtr nav_subscription_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_local_grid_;
  rclcpp::CallbackGroup::SharedPtr callback_group_nav_;

  // tf2 Listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // tf buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  // Transformer
  std::unique_ptr<aut_common::Transformer> transformer_;

  // Mutex
  std::mutex state_mtx_;

};

}  // namespace aut_local_planner

#endif  // AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_