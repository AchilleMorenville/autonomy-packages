#ifndef AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_
#define AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_

#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <aut_msgs/msg/local_grid.hpp>
#include <aut_msgs/msg/nav.hpp>
#include <aut_msgs/msg/nav_command.hpp>
// #include <aut_local_planner/grid.h>
#include <aut_local_planner/local_grid.h>

namespace aut_local_planner {

class LocalPlanner : public rclcpp::Node {

 public:
  explicit LocalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg);
  void NavCallBack(const aut_msgs::msg::Nav::SharedPtr nav_msg);

  // bool LocalPlanner::IsFreeLocalGrid(Eigen::Vector3f local_grid_to_position);

  // State

  bool initialized_;
  bool has_path_;

  int idx_current_node_;

  std::vector<Eigen::Vector3f> path_;
  // Grid grid_;
  LocalGrid local_grid_;

  // Publisher
  rclcpp::Publisher<aut_msgs::msg::NavCommand>::SharedPtr nav_command_publisher_;

  // Subscription
  rclcpp::Subscription<aut_msgs::msg::LocalGrid>::SharedPtr local_grid_subscription_;
  rclcpp::Subscription<aut_msgs::msg::Nav>::SharedPtr nav_subscription_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_local_grid_;
  rclcpp::CallbackGroup::SharedPtr callback_group_nav_;

  // tf2 Listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // tf buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // Mutex
  std::mutex state_mtx_;

};

}  // namespace aut_local_planner

#endif  // AUT_LOCAL_PLANNER_LOCAL_PLANNER_H_