#include "aut_local_planner/local_planner.h"

#include <mutex>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <aut_msgs/srv/graph_modif.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "aut_utils/utils.h"
#include "aut_common/transformer.h"
#include "aut_msgs/msg/local_grid.hpp"
#include "aut_msgs/srv/set_global_path.hpp"
#include "aut_msgs/msg/speed_command.hpp"

namespace aut_local_planner {

LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options)
    : Node("local_planner", options), a_star_(128, 128, 0.03f, 0.25f, 0.55f, 3.0f) {
  path_is_set_ = false;

  //tf
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  transformer_ = std::make_unique<aut_common::Transformer>(tf_buffer_);

  // Callback groups
  local_grid_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );
  set_global_path_callback_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  // Subscriptions
  rclcpp::SubscriptionOptions local_grid_options = rclcpp::SubscriptionOptions();
  local_grid_options.callback_group = local_grid_callback_group_;
  local_grid_subscription_ = this->create_subscription<aut_msgs::msg::LocalGrid>(
      "aut_spot/local_grid", 10, 
      std::bind(&LocalPlanner::LocalGridCallBack, this, std::placeholders::_1),
      local_grid_options
  );

  // Services
  set_global_path_service_ = this->create_service<aut_msgs::srv::SetGlobalPath>(
      "aut_local_planner/set_global_path", 
      std::bind(&LocalPlanner::SetGlobalPathService, this, std::placeholders::_1, 
                std::placeholders::_2),
      rmw_qos_profile_services_default,
      set_global_path_callback_group_
  );

  // Publisher
  occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("aut_local_planner/occupancy_grid", 10);
  speed_command_publisher_ = this->create_publisher<aut_msgs::msg::SpeedCommand>("aut_local_planner/speed_command", 10);
  
  // Service Clients
  goal_achieved_client_ = this->create_client<std_srvs::srv::Trigger>("aut_global_planner/goal_achieved");
  graph_modif_client_ = this->create_client<aut_msgs::srv::GraphModif>("aut_global_planner/graph_modif");
}

void LocalPlanner::LocalGridCallBack(
    const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg) {

  RCLCPP_INFO(this->get_logger(), "Received local grid");

  // occupancy_grid_publisher_->publish(LocalGridToOccupancyGrid(local_grid_msg->local_grid, local_grid_msg->pose));

  if (!path_is_set_) { return; }
  if (!transformer_->CanTransformMapToBaseLink(local_grid_msg->header.stamp)) { return; }

  Eigen::Matrix4f current_pose = transformer_->LookupTransformMapToBaseLink(local_grid_msg->header.stamp);

  std::vector<Eigen::Matrix4f> select_global_path;

  {
    std::lock_guard<std::mutex> global_path_lock(global_path_mtx_);
    auto closest_it = global_path_poses_.begin();
    float closest_dist = (current_pose.block<3, 1>(0, 3) - closest_it->block<3, 1>(0, 3)).norm();
    for (auto it = global_path_poses_.begin(); it != global_path_poses_.end(); ++it) {
      float dist = (current_pose.block<3, 1>(0, 3) - it->block<3, 1>(0, 3)).norm();
      if (dist < closest_dist) {
        closest_dist = dist;
        closest_it = it;
      }
    }

    select_global_path = std::vector<Eigen::Matrix4f>(closest_it, global_path_poses_.end());
  }

  nav_msgs::msg::OccupancyGrid map = LocalGridToOccupancyGrid(local_grid_msg->local_grid, local_grid_msg->pose);

  for (auto rit = select_global_path.rbegin(); rit != select_global_path.rend(); ++rit) {
    a_star_.SetGrid(local_grid_msg->local_grid, 
                    aut_utils::InverseTransformation(
                        aut_utils::TransformToMatrix(local_grid_msg->pose)), 
                    current_pose);
    Eigen::Matrix4f pose = *rit;
    if (!a_star_.SetTarget(pose)) { continue; }
    std::vector<Eigen::Vector2f> local_path;
    bool found_path = a_star_.GetPath(local_path);
    if (!found_path) { continue; }
    
    // Only for occupancy
    std::vector<std::pair<int, int>> path_idx;
    a_star_.FindPath(path_idx);
    for (std::pair<int, int>& p : path_idx) {
      map.data[p.first + 128 * p.second] = 100;
    }
    // --------

    if (local_path[local_path.size() - 1].norm() <= 0.15) { // if we are really close to the target we "must" select
      if (rit == select_global_path.rbegin()) {
        RCLCPP_INFO(this->get_logger(), "Goal achieved");
        auto goal_achieved_request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto goal_achieved_result = goal_achieved_client_->async_send_request(goal_achieved_request);
        path_is_set_ = false;
        {
          std::lock_guard<std::mutex> global_path_lock(global_path_mtx_);
          global_path_poses_.clear();
        }
        // Send goal achieved and reset
      } else {
        RCLCPP_INFO(this->get_logger(), "Stuck");
        auto graph_modif_request = std::make_shared<aut_msgs::srv::GraphModif::Request>();
        graph_modif_request->position_2.x = (*rit)(0, 3);
        graph_modif_request->position_2.y = (*rit)(1, 3);
        graph_modif_request->position_2.z = (*rit)(2, 3);

        graph_modif_request->position_1.x = (*(rit + 1))(0, 3);
        graph_modif_request->position_1.y = (*(rit + 1))(1, 3);
        graph_modif_request->position_1.z = (*(rit + 1))(2, 3);

        auto graph_modif_result = graph_modif_client_->async_send_request(graph_modif_request);
        path_is_set_ = false;
        {
          std::lock_guard<std::mutex> global_path_lock(global_path_mtx_);
          global_path_poses_.clear();
        }
      }
    } else {
      aut_msgs::msg::SpeedCommand speed_command;
      int look_idx = std::min(15, static_cast<int>(local_path.size() - 1));
      float angle = std::atan2(local_path[look_idx](1), local_path[look_idx](0));
      RCLCPP_INFO(this->get_logger(), "Angle : %f", angle);
      if (std::abs(angle) <= 20 * M_PI / 180.0f) {
        RCLCPP_INFO(this->get_logger(), "Go forward");
        float k = 2 * local_path[look_idx](1) / local_path[look_idx].squaredNorm();
        // float a_k = std::abs(k);
        // float v_xk = 0.4f;
        // float v_xd = 0.4f;
        // if (a_k > 1 / 0.5) {
        //   v_xk = std::max(0.2, 0.4 / (0.5 * 1 / a_k));
        // }
        // float d = local_grid_msg->local_grid[static_cast<int>(local_path[look_idx](0) / 0.03f) + 128 * static_cast<int>(local_path[look_idx](1) / 0.03f)];
        // if (d <= 0.55) {
        //   v_xd = std::max(0.2, 0.4f * (d - 0.25) / (0.55f - 0.25f));
        // }
        // speed_command.v_x = std::min(v_xk, v_xd);
        speed_command.v_x = 0.2;
        speed_command.v_t = speed_command.v_x * k;
      } else if (std::abs(angle) <= M_PI / 2.0f) {
        RCLCPP_INFO(this->get_logger(), "Rotate");
        speed_command.v_t = angle > 0 ? 0.2 : -0.2;
      } else {
        RCLCPP_INFO(this->get_logger(), "Reverse");
        if (local_grid_msg->local_grid[static_cast<int>(local_path[look_idx](0) / 0.03f) + 128 * static_cast<int>(local_path[look_idx](1) / 0.03f)] < 0.55f) {
          RCLCPP_INFO(this->get_logger(), "-> Stay reversed");
          Eigen::Vector2f direction = local_path[look_idx];
          Eigen::Affine2f af;
          af.linear() = Eigen::Rotation2Df(M_PI).toRotationMatrix();
          direction = af.inverse() * direction;
          angle = std::atan2(direction(1), direction(0));
          if (std::abs(angle) <= 20 * M_PI / 180.0f) {
            RCLCPP_INFO(this->get_logger(), "-> Continue backward");
            float k = 2 * local_path[look_idx](1) / local_path[look_idx].squaredNorm();
            float a_k = std::abs(k);
            float v_xk = 0.4f;
            float v_xd = 0.4f;
            if (a_k > 1 / 0.5) {
              v_xk = std::max(0.2, 0.4 / (0.5 * 1 / a_k));
            }
            float d = local_grid_msg->local_grid[static_cast<int>(local_path[look_idx](0) / 0.03f) + 128 * static_cast<int>(local_path[look_idx](1) / 0.03f)];
            if (d <= 0.55) {
              v_xd = std::max(0.2, 0.4f * (d - 0.25) / (0.55f - 0.25f));
            }
            speed_command.v_x = -0.2;
            speed_command.v_t = speed_command.v_x * k;
          } else {
            RCLCPP_INFO(this->get_logger(), "-> Rotate to backward");
            speed_command.v_t = angle > 0 ? 0.2 : -0.2;
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "-> Rotate");
          speed_command.v_t = angle > 0 ? 0.2 : -0.2;
        }
      }
      speed_command_publisher_->publish(speed_command);
    }
    // aut_msgs::msg::SpeedCommand command = GetSpeedCommand(local_path);
    break;
  }



  occupancy_grid_publisher_->publish(map);
}

aut_msgs::msg::SpeedCommand LocalPlanner::GetSpeedCommand(std::vector<Eigen::Vector2f>& local_path) {
  return aut_msgs::msg::SpeedCommand();
}

void LocalPlanner::SetGlobalPathService(
    const std::shared_ptr<aut_msgs::srv::SetGlobalPath::Request> request, 
    std::shared_ptr<aut_msgs::srv::SetGlobalPath::Response> response) {
  path_is_set_ = false;
  {
    std::lock_guard<std::mutex> global_path_lock(global_path_mtx_);
    global_path_poses_.clear();
    for (const geometry_msgs::msg::PoseStamped& pose_stamped : request->global_path.poses) {
      Eigen::Vector3f t_pose(pose_stamped.pose.position.x, 
                            pose_stamped.pose.position.y, 
                            pose_stamped.pose.position.z);
      Eigen::Quaternionf q_pose(pose_stamped.pose.orientation.w, 
                                pose_stamped.pose.orientation.x, 
                                pose_stamped.pose.orientation.y, 
                                pose_stamped.pose.orientation.z);
      Eigen::Matrix4f m_pose = Eigen::Matrix4f::Identity();
      m_pose.block<3, 1>(0, 3) = t_pose;
      m_pose.block<3, 3>(0, 0) = q_pose.toRotationMatrix();
      global_path_poses_.push_back(m_pose);
    }
    if (!global_path_poses_.empty()) { path_is_set_ = true; }
  }
  response->success = true;
}

nav_msgs::msg::OccupancyGrid LocalPlanner::LocalGridToOccupancyGrid(std::vector<float>& local_grid, geometry_msgs::msg::Transform& base_link_to_local_grid) {
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

}  // namespace aut_local_planner

// #include "aut_local_planner/local_planner.h"

// #include <memory>
// #include <cmath>
// #include <limits>
// #include <iostream>

// #include <rclcpp/rclcpp.hpp>
// #include <tf2_ros/buffer.h>
// #include <tf2_ros/transform_listener.h>

// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <geometry_msgs/msg/transform.hpp>
// #include <visualization_msgs/msg/marker.hpp>
// #include <geometry_msgs/msg/point.hpp>

// #include <Eigen/Core>

// #include "aut_msgs/msg/local_grid.hpp"
// #include "aut_msgs/msg/nav.hpp"
// #include "aut_msgs/msg/nav_command.hpp"
// #include "aut_msgs/msg/nav_modif.hpp"

// #include "aut_utils/utils.h"
// #include "aut_local_planner/local_grid.h"
// #include "aut_local_planner/utils.h"
// #include "aut_common/transformer.h"

// namespace aut_local_planner {

// LocalPlanner::LocalPlanner(const rclcpp::NodeOptions& options)
//     : Node("local_planner", options), local_grid_() {

//   tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//   tf_buffer_->setUsingDedicatedThread(true);

//   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//   transformer_ = std::make_unique<aut_common::Transformer>(tf_buffer_);

//   callback_group_local_grid_ = this->create_callback_group(
//       rclcpp::CallbackGroupType::MutuallyExclusive
//   );

//   rclcpp::SubscriptionOptions local_grid_options = rclcpp::SubscriptionOptions();
//   local_grid_options.callback_group = callback_group_local_grid_;

//   local_grid_subscription_ = this->create_subscription<aut_msgs::msg::LocalGrid>(
//     "aut_spot/local_grid", 10, 
//     std::bind(&LocalPlanner::LocalGridCallBack, this, std::placeholders::_1),
//     local_grid_options
//   );

//   callback_group_nav_ = this->create_callback_group(
//       rclcpp::CallbackGroupType::MutuallyExclusive
//   );

//   rclcpp::SubscriptionOptions nav_options = rclcpp::SubscriptionOptions();
//   nav_options.callback_group = callback_group_nav_;

//   nav_subscription_ = this->create_subscription<aut_msgs::msg::Nav>(
//     "aut_global_planner/nav", 10, 
//     std::bind(&LocalPlanner::NavCallBack, this, std::placeholders::_1),
//     nav_options
//   );

//   nav_command_publisher_ = this->create_publisher<aut_msgs::msg::NavCommand>("aut_local_planner/nav_command", 10);
//   occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("aut_local_planner/occupancy_grid", 10);
//   nav_modif_publisher_ = this->create_publisher<aut_msgs::msg::NavModif>("aut_local_planner/nav_modif", 10);
//   marker_command_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("aut_local_planner/marker_command", 10); 
// }

// void LocalPlanner::LocalGridCallBack(const aut_msgs::msg::LocalGrid::SharedPtr local_grid_msg) {
//   RCLCPP_INFO(this->get_logger(), "Received local grid");

//   if (!transformer_->CanTransformMapToBaseLink(local_grid_msg->header.stamp)) {
//     RCLCPP_INFO(this->get_logger(), "Cannot get transform map -> base_link");
//     return;
//   }

//   Eigen::Matrix4f m_local_grid_to_base_link = aut_utils::InverseTransformation(aut_utils::TransformToMatrix(local_grid_msg->pose));
//   Eigen::Matrix4f m_map_to_base_link = transformer_->LookupTransformMapToBaseLink(local_grid_msg->header.stamp);

//   local_grid_.SetLocalGrid(local_grid_msg->local_grid, m_local_grid_to_base_link, m_map_to_base_link);
// }

// void LocalPlanner::NavCallBack(const aut_msgs::msg::Nav::SharedPtr nav_msg) {
//   RCLCPP_INFO(this->get_logger(), "Received nav_msg");
// }

// }  // namespace aut_local_planner