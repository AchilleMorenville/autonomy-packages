#include "aut_local_planner/local_planner.h"

#include <memory>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include "aut_msgs/msg/local_grid.hpp"
#include "aut_msgs/msg/nav.hpp"
#include "aut_msgs/msg/nav_command.hpp"
// #include "aut_msgs/msg/nav_modif.hpp"
#include "aut_utils/utils.h"

namespace aut_local_planner {

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options)
    : Node("local_planner", options), local_grid_(0.25f, 0.75f) {

  has_path_ = false;
  initialized_ = false;
  idx_current_node_ = -1;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

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

  // nav_modif_publisher_ = this->create_publisher<aut_msgs::msg::NavModif>("aut_local_planner/nav_modif", 10);
}

void LocalPlanner::LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg) {
  RCLCPP_INFO(this->get_logger(), "Received local grid");
  (void)local_grid_msg;

  state_mtx_.lock();
  if (!has_path_) {
    state_mtx_.unlock();
    return;
  }
  state_mtx_.unlock();

  if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
    return;
  }

  geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
  Eigen::Matrix4f map_to_base_link = aut_utils::TransformToMatrix(trans);
  Eigen::Matrix4f base_link_to_local_grid = aut_utils::TransformToMatrix(local_grid_msg->pose);

  local_grid_.AddLocalGrid(local_grid_msg->local_grid, map_to_base_link, base_link_to_local_grid);

  // Search for new target
  int furthest_accessible = idx_current_node_;
  for (int i = idx_current_node_ + 1; i < (int) path_.size(); ++i) {
    if (local_grid_.IsInside(path_[i])) {
      Eigen::Vector2f direction;
      if (local_grid_.FindPath(path_[i], direction)) {
        furthest_accessible = i;
      }
    } else {
      break;
    }
  }

  // If no new target and current one is -1 (start) -> abort and send impossible
  if (furthest_accessible < 0) {
    state_mtx_.lock();
    has_path_ = false;
    state_mtx_.unlock();

    RCLCPP_INFO(this->get_logger(), "Not access to start");

    // Send to global that cannot access start
    return;
  }

  // No new target and really close to current one -> abort and send impossible to move forward
  if (idx_current_node_ < (int) path_.size() - 1 && furthest_accessible == idx_current_node_ && (map_to_base_link.block<3, 1>(0, 3) - path_[idx_current_node_]).norm() <= 0.2) {
    state_mtx_.lock();
    has_path_ = false;
    state_mtx_.unlock();

    RCLCPP_INFO(this->get_logger(), "Not access to idx_current_node_ + 1");

    // Send to global that cannot access idx_current_node_ + 1
    return;
  }

  if (idx_current_node_ == (int) path_.size() - 1 && (map_to_base_link.block<3, 1>(0, 3) - path_[idx_current_node_]).norm() <= 0.2) {
    state_mtx_.lock();
    has_path_ = false;
    state_mtx_.unlock();

    RCLCPP_INFO(this->get_logger(), "Goal accessed");

    return;
  }

  idx_current_node_ = furthest_accessible;

  // Check for current path.
  Eigen::Vector2f direction;
  bool found_path = local_grid_.FindPath(path_[idx_current_node_], direction);

  if (!found_path) {
    state_mtx_.lock();
    has_path_ = false;
    state_mtx_.unlock();

    // Send to global that cannot access idx_current_node_
    RCLCPP_INFO(this->get_logger(), "Not access to idx_current_node_");
    return;
  }

  Eigen::Vector2f body(1, 0);
  float angle = std::acos( (body.dot(direction)) / (body.norm() * direction.norm())) * 180 / M_PI;

  aut_msgs::msg::NavCommand nav_command_msg;

  if (std::abs(angle) <= 20.0) {
    nav_command_msg.angle = angle;
    nav_command_msg.x = direction(0);
    nav_command_msg.y = direction(1);
  } else {
    nav_command_msg.angle = angle;
    nav_command_msg.x = 0;
    nav_command_msg.y = 0;
  }

  // nav_command_msg.x = direction(0);
  // nav_command_msg.y = direction(1);
  // nav_command_msg.angle = angle;

  nav_command_publisher_->publish(nav_command_msg);
  // Send to Spot current goal
}

// void LocalPlanner::LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg) {
//   RCLCPP_INFO(this->get_logger(), "Received local grid");
//   (void)local_grid_msg;

//   state_mtx_.lock();
//   if (!has_path_) {
//     state_mtx_.unlock();
//     return;
//   }
//   state_mtx_.unlock();

//   if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
//     return;
//   }

//   geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
//   Eigen::Matrix4f map_to_base_link = aut_utils::TransformToMatrix(trans);
//   Eigen::Matrix4f base_link_to_local_grid = aut_utils::TransformToMatrix(local_grid_msg->pose);

//   Eigen::Matrix4f local_grid_to_map = aut_utils::InverseTransformation(map_to_base_link * base_link_to_local_grid);

//   if (!initialized_) {
//     int furthest_node = -1;
//     for (int i = 0; i < 3 && i < (int) path_.size(); ++i) {
//       Eigen::Matrix4f map_to_node = Eigen::Matrix4f::Identity();
//       map_to_node.block<3, 1>(0, 3) = path_[i];
//       Eigen::Matrix4f local_grid_to_node = local_grid_to_map * map_to_node;
//       if (IsFreeLocalGrid(local_grid_msg->local_grid, local_grid_to_node.block<3, 1>(0, 3))) {
//         furthest_node = i;
//       }
//     }

//     // Init grid to furthest_node
//     idx_current_node_ = furthest_node;
//     grid_.Initialize(path_[furthest_node]);
//     initialized_ = true;
//   }

//   // Add local grid to grid.
//   grid_.AddLocalGrid(local_grid_msg->local_grid, map_to_base_link, base_link_to_local_grid);

//   // Check for next current goals if they exists and are in the explored map and are free and update the grid
//   int new_furthest_node = -1;
//   for (int i = idx_current_node_; i < idx_current_node_ + 3 && i < (int) path_.size(); ++i) {
//     if (grid_.IsFreeAndSeen(path_[i])) {
//       new_furthest_node = i;
//     }
//   }

//   // If new goal -> move grid
//   if (new_furthest_node > idx_current_node_) {
//     grid_.Move(path_[new_furthest_node]);
//   }

//   // Search for path, if none exists send to global that there is a problem
//   Eigen::Vector2f direction;
//   bool found_path = grid_.GetDirection(map_to_base_link, direction);

//   if (!found_path) {
//     // Send to global that there is a problem
//   }

//   // Send command

// }

// bool LocalPlanner::IsFreeLocalGrid(std::vector<float>& local_grid, Eigen::Vector3f local_grid_to_position) {
//   int idx_1 = (int) (local_grid_to_position(0) / 0.03f);
//   int idx_2 = (int) (local_grid_to_position(1) / 0.03f);
//   if (idx_1 < 0 || idx_2 < 0 || idx_1 >= 128 || idx_2 >= 128) {
//     return false;
//   }
//   int idx = idx_1 + 128 * idx_2;
//   return local_grid[idx] > min_dist_;
// }

void LocalPlanner::NavCallBack(const aut_msgs::msg::Nav::SharedPtr nav_msg) {
  RCLCPP_INFO(this->get_logger(), "Received nav_msg");
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (nav_msg->positions.size() <= 0) {
    has_path_ = false;
    return;
  }

  if (!has_path_) {
    has_path_ = true;
  }

  path_.clear();
  for (int i = 0; i < (int) nav_msg->positions.size(); i++) {
    Eigen::Vector3f position(nav_msg->positions[i].x, nav_msg->positions[i].y, nav_msg->positions[i].z);
    path_.push_back(position);
  }

  idx_current_node_ = -1;
}

}  // namespace aut_local_planner