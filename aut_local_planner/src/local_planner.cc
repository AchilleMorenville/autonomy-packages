#include "aut_local_planner/local_planner.h"

#include <memory>
#include <cmath>
#include <limits>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Core>

#include "aut_msgs/msg/local_grid.hpp"
#include "aut_msgs/msg/nav.hpp"
#include "aut_msgs/msg/nav_command.hpp"
#include "aut_msgs/msg/nav_modif.hpp"

#include "aut_utils/utils.h"
#include "aut_local_planner/local_grid.h"
#include "aut_local_planner/utils.h"
#include "aut_common/transformer.h"

namespace aut_local_planner {

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options)
    : Node("local_planner", options), local_grid_() {

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  transformer_ = std::make_unique<aut_common::Transformer>(tf_buffer_);

  callback_group_local_grid_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions local_grid_options = rclcpp::SubscriptionOptions();
  local_grid_options.callback_group = callback_group_local_grid_;

  local_grid_subscription_ = this->create_subscription<aut_msgs::msg::LocalGrid>(
    "aut_spot/local_grid", 10, 
    std::bind(&LocalPlanner::LocalGridCallBack, this, std::placeholders::_1),
    local_grid_options
  );

  callback_group_nav_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions nav_options = rclcpp::SubscriptionOptions();
  nav_options.callback_group = callback_group_nav_;

  nav_subscription_ = this->create_subscription<aut_msgs::msg::Nav>(
    "aut_global_planner/nav", 10, 
    std::bind(&LocalPlanner::NavCallBack, this, std::placeholders::_1),
    nav_options
  );

  nav_command_publisher_ = this->create_publisher<aut_msgs::msg::NavCommand>("aut_local_planner/nav_command", 10);
  occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("aut_local_planner/occupancy_grid", 10);
  nav_modif_publisher_ = this->create_publisher<aut_msgs::msg::NavModif>("aut_local_planner/nav_modif", 10);
  marker_command_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("aut_local_planner/marker_command", 10); 
}

void LocalPlanner::LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg) {
  RCLCPP_INFO(this->get_logger(), "Received local grid");

  if (!transformer_->CanTransformMapToBaseLink(local_grid_msg->header.stamp)) {
    return;
  }

  Eigen::Matrix4f m_local_grid_to_base_link = aut_utils::InverseTransformation(aut_utils::TransformToMatrix(local_grid_msg->pose));
  Eigen::Matrix4f m_map_to_base_link = transformer_->LookupTransformMapToBaseLink(local_grid_msg->header.stamp);

  local_grid_.SetLocalGrid(local_grid_msg->local_grid, m_local_grid_to_base_link, m_map_to_base_link);
}

void LocalPlanner::NavCallBack(const aut_msgs::msg::Nav::SharedPtr nav_msg) {
  RCLCPP_INFO(this->get_logger(), "Received nav_msg");
}

}  // namespace aut_local_planner