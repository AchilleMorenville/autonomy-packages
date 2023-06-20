#ifndef AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_
#define AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <aut_common/graph.h>

namespace aut_global_planner {

class GlobalPlanner : public rclcpp::Node {

 public:
  explicit GlobalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:

  int goal;
  int start;

  aut_common::Graph nav_graph_;

  visualization_msgs::msg::MarkerArray last_marker_array;

  // tf2 Listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // tf buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;

  void timer_callback();
};

}  // namespace aut_global_planner

#endif  // AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_