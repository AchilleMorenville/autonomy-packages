#include "aut_global_planner/global_planner.h"

#include <memory>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform.h>

#include <Eigen/Core>

#include <aut_common/graph.h>
#include <aut_utils/utils.h>

namespace aut_global_planner {

GlobalPlanner::GlobalPlanner(const rclcpp::NodeOptions& options)
    : Node("global_planner", options) {

  goal = -1;
  start = -1;

  std::string graph_path("old/data/graph.txt");
  nav_graph_.LoadFile(graph_path);
  nav_graph_.Simplify(1.0);
  nav_graph_.TidyGraph();

  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose(0, 3) = 0.21;
  pose(1, 3) = -8.4;
  pose(2, 3) = -2.8;
  goal = nav_graph_.ClosestNode(pose);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aut_global_planner/graph", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&GlobalPlanner::timer_callback, this));
}

void GlobalPlanner::timer_callback() {
  if (goal < 0) {
    return;
  }

  if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
    return;
  }

  geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
  Eigen::Matrix4f map_tform_base_link = aut_utils::TransformToMatrix(
    trans
  );

  int new_start = nav_graph_.ClosestNode(map_tform_base_link);

  if (new_start == start) {
    marker_array_publisher_->publish(last_marker_array);
    return;
  }

  start = new_start;

  std::vector<int> path;
  nav_graph_.AStar(start, goal, path);

  last_marker_array = nav_graph_.GetMarkerArrayWithPath(path);

  marker_array_publisher_->publish(last_marker_array);
}

}  // namespace aut_global_planner