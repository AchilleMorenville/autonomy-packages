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

#include <Eigen/Core>

#include "aut_msgs/msg/local_grid.hpp"
#include "aut_msgs/msg/nav.hpp"
#include "aut_msgs/msg/nav_command.hpp"
#include "aut_msgs/msg/nav_modif.hpp"
#include "aut_utils/utils.h"

namespace aut_local_planner {

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options)
    : Node("local_planner", options), local_grid_(0.25f, 0.70f, 0.20f, 3.0f) {

  has_path_ = false;
  initialized_ = false;
  idx_current_node_ = -1;
  count_preceeding_stop_ = 0;
  preceeding_stop_ = -1;

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
  occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("aut_local_planner/occupancy_grid", 10);
  nav_modif_publisher_ = this->create_publisher<aut_msgs::msg::NavModif>("aut_local_planner/nav_modif", 10);
}

void LocalPlanner::LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg) {
  RCLCPP_INFO(this->get_logger(), "Received local grid");

  // Eigen::Matrix4f base_link_to_local_grid_2 = aut_utils::TransformToMatrix(local_grid_msg->pose);
  // local_grid_.AddLocalGrid(local_grid_msg->local_grid, Eigen::Matrix4f::Identity(), base_link_to_local_grid_2);

  // // nav_msgs::msg::OccupancyGrid occupancy_grid_msg = LocalGridToOccupancyGrid(local_grid_msg->local_grid, local_grid_msg->pose);
  // nav_msgs::msg::OccupancyGrid occupancy_grid_msg = local_grid_.LocalGridToOccupancyGrid();
  // occupancy_grid_publisher_->publish(occupancy_grid_msg);

  state_mtx_.lock();
  if (!has_path_) {
    state_mtx_.unlock();
    RCLCPP_INFO(this->get_logger(), "Has no path");
    return;
  }
  state_mtx_.unlock();

  deque_local_grid_msgs_.push_back(*local_grid_msg);

  std::string* err1 = new std::string();
  if (!tf_buffer_->canTransform("map", "base_link", deque_local_grid_msgs_.front().header.stamp, rclcpp::Duration::from_seconds(0.05), err1)) {
    RCLCPP_INFO(this->get_logger(), "No data for map_to_base_link");
    RCLCPP_INFO(this->get_logger(), (*err1).c_str());
    return;

    if ((int) deque_local_grid_msgs_.size() > 2) {
      deque_local_grid_msgs_.pop_front();
    }
  }

  aut_msgs::msg::LocalGrid current_local_grid_msg = deque_local_grid_msgs_.front();
  deque_local_grid_msgs_.pop_front();

  geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", current_local_grid_msg.header.stamp).transform;
  Eigen::Matrix4f map_to_base_link = aut_utils::TransformToMatrix(trans);

  geometry_msgs::msg::Transform trans_grav = tf_buffer_->lookupTransform("base_link", "gravity", current_local_grid_msg.header.stamp).transform;
  Eigen::Matrix4f base_link_to_gravity = aut_utils::TransformToMatrix(trans_grav);

  Eigen::Matrix4f base_link_to_local_grid = aut_utils::TransformToMatrix(current_local_grid_msg.pose);

  Eigen::Matrix4f local_grid_to_gravity = aut_utils::InverseTransformation(base_link_to_local_grid) * base_link_to_gravity;

  std::cout << local_grid_to_gravity << std::endl;

  local_grid_.AddLocalGrid(current_local_grid_msg.local_grid, map_to_base_link, base_link_to_local_grid);

  RCLCPP_INFO(this->get_logger(), "Find closest node");
  float closest_dist = std::numeric_limits<float>::max();
  int closest_idx = -1;
  for (int i = 0; i < (int) path_.size(); ++i) {
    float dist = (map_to_base_link.block<3, 1>(0, 3) - path_[i]).norm();
    if (dist < closest_dist) {
      closest_idx = i;
    }
  }

  // Eigen::Matrix4f gravity_to_map = aut_utils::InverseTransformation(map_to_base_link * base_link_to_gravity);
  bool down_stairs_ahead = false;
  bool up_stairs_ahead = false;
  for (int i = closest_idx; i < ((int) path_.size() - 1) && i < closest_idx + 5; ++i) {
    float delta_z = path_[i + 1](2) - path_[i](2);
    float dist_xyz = (path_[i + 1] - path_[i]).norm();
    if (dist_xyz > 0.05f) {
      float stair_angle = std::asin(delta_z / dist_xyz) * 180.0f / M_PI;
      if (stair_angle < -20.0f) {
        down_stairs_ahead = true;
      } else if (stair_angle > 20.0f) {
        up_stairs_ahead = true;
      }
    }
  }

  bool is_rotation_safe = local_grid_.IsCurrentPositionRotationSafe();


  Eigen::Vector3f z_gravity(0.0, 0.0, 1.0);

  Eigen::Matrix4f base_link_to_z = Eigen::Matrix4f::Identity();
  base_link_to_z(2, 3) = 1;

  Eigen::Vector3f z_base_link = (aut_utils::InverseTransformation(base_link_to_gravity) * base_link_to_z).block<3, 1>(0, 3);

  float gravity_angle = std::acos( z_gravity.dot(z_base_link) / (z_gravity.norm() * z_base_link.norm()) ) * 180.0f / M_PI;

  bool is_on_slope = std::abs(gravity_angle) > 20.0f;

  RCLCPP_INFO(this->get_logger(), "Create target vectors");
  std::vector<int> targets_idx;
  std::vector<Eigen::Vector3f> targets_position;
  for (int i = std::min(closest_idx - 3, 0); i < ((int) path_.size()) && i < closest_idx + 3; ++i) {
    if (local_grid_.IsInGrid(path_[i])) {
      targets_position.push_back(path_[i]);
      targets_idx.push_back(i);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Start Command");

  Eigen::Vector2f direction;
  int result = local_grid_.GetDirection(targets_position, direction);

  if (result < 0) {
    RCLCPP_INFO(this->get_logger(), "Impossible");
    // send impossible
    aut_msgs::msg::NavModif nav_modif_msg;
    nav_modif_msg.inaccessible_node_in_path = -1;
    nav_modif_publisher_->publish(nav_modif_msg);
    return;
  }

  if (direction(0) == 0.0f && direction(1) == 0.0f && targets_idx[result] < (int) path_.size() - 1) {
    // send next to result impossible

    if (preceeding_stop_ == targets_idx[result]) {
      count_preceeding_stop_++;
    } else {
      count_preceeding_stop_ = 0;
      preceeding_stop_ = targets_idx[result];
    }

    if (count_preceeding_stop_ > 10) {
      aut_msgs::msg::NavModif nav_modif_msg;
      nav_modif_msg.inaccessible_node_in_path = targets_idx[result] + 1;
      nav_modif_publisher_->publish(nav_modif_msg);
    }

    return;
  }

  // float heading_angle = std::atan2(direction(1), direction(0));
  float heading_angle = GetAngle(Eigen::Vector2f(1, 0), direction) * 180.0f / M_PI;  // Relative to the front of the robot
  
  RCLCPP_INFO(this->get_logger(), "Heading angle: %f", heading_angle);

  bool is_backward = std::abs(heading_angle) > 90.0f;

  // Send command
  RCLCPP_INFO(this->get_logger(), "Is backward: %d", is_backward);
  RCLCPP_INFO(this->get_logger(), "Down stairs ahead: %d", down_stairs_ahead);
  RCLCPP_INFO(this->get_logger(), "Up stairs ahead: %d", up_stairs_ahead);
  RCLCPP_INFO(this->get_logger(), "Is Rotation Safe: %d", is_rotation_safe);
  RCLCPP_INFO(this->get_logger(), "Is On Slope: %d", is_on_slope);

  RCLCPP_INFO(this->get_logger(), "Found direction dx: %f, dy: %f", direction(0), direction(1));

  nav_command_publisher_->publish(CreateCommand(direction, is_backward, is_rotation_safe, is_on_slope, up_stairs_ahead, down_stairs_ahead));

  // nav_msgs::msg::OccupancyGrid occupancy_grid_msg = LocalGridToOccupancyGrid(local_grid_msg->local_grid, local_grid_msg->pose);
  nav_msgs::msg::OccupancyGrid occupancy_grid_msg = local_grid_.LocalGridToOccupancyGrid();
  occupancy_grid_publisher_->publish(occupancy_grid_msg);

  // // Search for new target
  // int furthest_accessible = idx_current_node_;
  // for (int i = idx_current_node_ + 1; i < (int) path_.size(); ++i) {
  //   if (local_grid_.IsInside(path_[i])) {
  //     Eigen::Vector2f direction;
  //     if (local_grid_.FindPath(path_[i], direction)) {
  //       furthest_accessible = i;
  //     }
  //   } else {
  //     break;
  //   }
  // }

  // // If no new target and current one is -1 (start) -> abort and send impossible
  // if (furthest_accessible < 0) {
  //   state_mtx_.lock();
  //   has_path_ = false;
  //   state_mtx_.unlock();

  //   RCLCPP_INFO(this->get_logger(), "Not access to start");

  //   // Send to global that cannot access start
  //   return;
  // }

  // // No new target and really close to current one -> abort and send impossible to move forward
  // if (idx_current_node_ < (int) path_.size() - 1 && furthest_accessible == idx_current_node_ && (map_to_base_link.block<3, 1>(0, 3) - path_[idx_current_node_]).norm() <= 0.2) {
  //   state_mtx_.lock();
  //   has_path_ = false;
  //   state_mtx_.unlock();

  //   RCLCPP_INFO(this->get_logger(), "Not access to idx_current_node_ + 1");

  //   // Send to global that cannot access idx_current_node_ + 1
  //   return;
  // }

  // if (idx_current_node_ == (int) path_.size() - 1 && (map_to_base_link.block<3, 1>(0, 3) - path_[idx_current_node_]).norm() <= 0.2) {
  //   state_mtx_.lock();
  //   has_path_ = false;
  //   state_mtx_.unlock();

  //   RCLCPP_INFO(this->get_logger(), "Goal accessed");

  //   return;
  // }

  // idx_current_node_ = furthest_accessible;

  // // Check for current path.
  // Eigen::Vector2f direction;
  // bool found_path = local_grid_.FindPath(path_[idx_current_node_], direction);

  // if (!found_path) {
  //   state_mtx_.lock();
  //   has_path_ = false;
  //   state_mtx_.unlock();

  //   // Send to global that cannot access idx_current_node_
  //   RCLCPP_INFO(this->get_logger(), "Not access to idx_current_node_");
  //   return;
  // }

  // Eigen::Vector2f body(1, 0);
  // float angle = std::acos( (body.dot(direction)) / (body.norm() * direction.norm())) * 180 / M_PI;

  // aut_msgs::msg::NavCommand nav_command_msg;

  // if (std::abs(angle) <= 20.0) {
  //   nav_command_msg.angle = angle;
  //   nav_command_msg.x = direction(0);
  //   nav_command_msg.y = direction(1);
  // } else {
  //   nav_command_msg.angle = angle;
  //   nav_command_msg.x = 0;
  //   nav_command_msg.y = 0;
  // }

  // // nav_command_msg.x = direction(0);
  // // nav_command_msg.y = direction(1);
  // // nav_command_msg.angle = angle;

  // nav_command_publisher_->publish(nav_command_msg);
  // // Send to Spot current goal
}

float LocalPlanner::GetAngle(Eigen::Vector2f body, Eigen::Vector2f target) {

  float theta = std::acos( (body.dot(target)) / (body.norm() * target.norm()));

  if (target(1) < 0 && body(0) > 0) {
    theta = - theta;
  } else if (target(1) > 0 && body(0) < 0) {
    theta = - theta;
  }
  return theta;
}

aut_msgs::msg::NavCommand LocalPlanner::CreateCommand(Eigen::Vector2f direction, bool is_backward, bool is_rotation_safe, bool is_on_slope, bool up_stairs_ahead, bool down_stairs_ahead) {

  aut_msgs::msg::NavCommand nav_command_msg;

  float heading_angle = GetAngle(Eigen::Vector2f(1, 0), direction);
  if (!is_backward && !down_stairs_ahead) {
    RCLCPP_INFO(this->get_logger(), "Continue Forward");
    heading_angle = GetAngle(Eigen::Vector2f(1, 0), direction);
  } else if (!is_on_slope && is_backward && is_rotation_safe && !down_stairs_ahead) {
    // Rotate to be forward
    RCLCPP_INFO(this->get_logger(), "Rotate to be forward");
    std::cout << "Rotate to be forward\n";
    heading_angle = GetAngle(Eigen::Vector2f(1, 0), direction);
  } else if (is_backward && !up_stairs_ahead) {
    std::cout << "Continue backward\n";
    heading_angle = GetAngle(Eigen::Vector2f(-1, 0), direction);
  } else if (!is_on_slope && !is_backward && is_rotation_safe && !up_stairs_ahead && down_stairs_ahead) {
    std::cout << "Rotate to be backward\n";
    heading_angle = GetAngle(Eigen::Vector2f(-1, 0), direction);
  } else {

    nav_command_msg.angle = 0.0f;
    nav_command_msg.x = 0.0f;
    nav_command_msg.y = 0.0f;
    nav_command_msg.max_speed = 0.2;

    return nav_command_msg;

  }

  if (std::abs(heading_angle) * 180.0f / M_PI <= 20) {
    // If the robot is mostly forward then continue to go forward
    nav_command_msg.angle = heading_angle;
    nav_command_msg.x = direction(0);
    nav_command_msg.y = direction(1);
    nav_command_msg.max_speed = std::abs(heading_angle) * 180.0f / M_PI <= 10 ? 0.4 : 0.2;
  } else {
    // The robot is not forward enough, only rotate 
    nav_command_msg.angle = heading_angle;
    nav_command_msg.x = 0.0f;
    nav_command_msg.y = 0.0f;
    nav_command_msg.max_speed = 0.2;
  }

  return nav_command_msg;

  // if ((!is_backward && !down_stairs_ahead) || (!is_on_slope && is_backward && is_rotation_safe && !down_stairs_ahead)) {
  //   // There is no down stairs ahead and the robot is going forward -> it can continue forward
  //   // Or the robot is ok and can turn back forward
  //   float heading_angle = std::atan2(direction(1), direction(0));

  //   RCLCPP_INFO(this->get_logger(), "Continue Forward");

  //   if (std::abs(heading_angle) * 180.0f / M_PI <= 20) {
  //     // If the robot is mostly forward then continue to go forward
  //     nav_command_msg.angle = heading_angle;
  //     nav_command_msg.x = direction(0);
  //     nav_command_msg.y = direction(1);
  //     nav_command_msg.max_speed = std::abs(heading_angle) * 180.0f / M_PI <= 10 ? 0.4 : 0.2;
  //   } else {
  //     // The robot is not forward enough, only rotate 
  //     nav_command_msg.angle = heading_angle;
  //     nav_command_msg.x = 0.0f;
  //     nav_command_msg.y = 0.0f;
  //     nav_command_msg.max_speed = 0.2;
  //   }
  // } else if ((is_backward && !up_stairs_ahead) || (!is_on_slope && !is_backward && is_rotation_safe && !up_stairs_ahead && down_stairs_ahead)) {
  //   // If the robot is backward and there is no upstairs ahead then continue backward
  //   // if the robot is not on a slope, is not backward, can rotate, with down stairs but without down stairs then rotates
  //   float heading_angle = std::atan2(-direction(1), -direction(0)); // TODO: Check if that works

  //   RCLCPP_INFO(this->get_logger(), "Continue Backward");

  //   if (std::abs(heading_angle) * 180.0f / M_PI <= 20) {
  //     // If the robot is mostly forward then continue to go forward
  //     nav_command_msg.angle = heading_angle;
  //     nav_command_msg.x = direction(0);
  //     nav_command_msg.y = direction(1);
  //     nav_command_msg.max_speed = std::abs(heading_angle) * 180.0f / M_PI <= 10 ? 0.4 : 0.2;
  //   } else {
  //     // The robot is not forward enough, only rotate 
  //     nav_command_msg.angle = heading_angle;
  //     nav_command_msg.x = 0.0f;
  //     nav_command_msg.y = 0.0f;
  //     nav_command_msg.max_speed = 0.2;
  //   }
  // } else {
  //   nav_command_msg.angle = 0.0f;
  //   nav_command_msg.x = 0.0f;
  //   nav_command_msg.y = 0.0f;
  //   nav_command_msg.max_speed = 0.2;
  // }

  return nav_command_msg;
}

nav_msgs::msg::OccupancyGrid LocalPlanner::LocalGridToOccupancyGrid(std::vector<float> local_grid, geometry_msgs::msg::Transform base_link_to_local_grid) {
  nav_msgs::msg::OccupancyGrid occupancy_grid;

  for (int i = 0; i < (int) local_grid.size(); ++i) {
    if (local_grid[i] < 0.25) {
      occupancy_grid.data.push_back(100);
    } else {
      occupancy_grid.data.push_back(0);
    }
  }

  occupancy_grid.header.frame_id = std::string("base_link");

  occupancy_grid.info.resolution = 0.03;
  occupancy_grid.info.width = 128;
  occupancy_grid.info.height = 128;
  occupancy_grid.info.origin.position.x = base_link_to_local_grid.translation.x;
  occupancy_grid.info.origin.position.y = base_link_to_local_grid.translation.y;
  occupancy_grid.info.origin.position.z = base_link_to_local_grid.translation.z;

  occupancy_grid.info.origin.orientation.x = base_link_to_local_grid.rotation.x;
  occupancy_grid.info.origin.orientation.y = base_link_to_local_grid.rotation.y;
  occupancy_grid.info.origin.orientation.z = base_link_to_local_grid.rotation.z;
  occupancy_grid.info.origin.orientation.w = base_link_to_local_grid.rotation.w;

  return occupancy_grid;
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
  preceeding_stop_ = -1;
  count_preceeding_stop_ = 0;
}

}  // namespace aut_local_planner