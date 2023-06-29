#include "aut_global_planner/global_planner.h"

#include <memory>
#include <chrono>
#include <string>
#include <thread>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform.h>
#include <geometry_msgs/msg/point.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include <aut_common/graph.h>
#include <aut_utils/utils.h>
#include <aut_msgs/action/navigate_to_goal.hpp>
#include <aut_msgs/msg/nav.hpp>
#include <aut_msgs/msg/nav_modif.hpp>

namespace aut_global_planner {

GlobalPlanner::GlobalPlanner(const rclcpp::NodeOptions& options)
    : Node("global_planner", options) {

  initialized_ = false;
  goal_is_set_ = false;
  nav_graph_has_changed_ = false;

  // std::string graph_path("old/data/graph.txt");
  // nav_graph_.LoadFile(graph_path);
  // nav_graph_.Simplify(1.0);
  // nav_graph_.TidyGraph();

  // Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  // pose(0, 3) = 0.21;
  // pose(1, 3) = -8.4;
  // pose(2, 3) = -2.8;
  // goal = nav_graph_.ClosestNode(pose);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);

  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aut_global_planner/graph", 10);
  nav_publisher_ = this->create_publisher<aut_msgs::msg::Nav>("aut_global_planner/nav", 10);

  action_server_ = rclcpp_action::create_server<aut_msgs::action::NavigateToGoal>(
      this,
      "aut_global_planner/navigate_to_goal",
      std::bind(&GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GlobalPlanner::handle_cancel, this, std::placeholders::_1),
      std::bind(&GlobalPlanner::handle_accepted, this, std::placeholders::_1)
  );

  load_map_service_ = this->create_service<aut_msgs::srv::LoadMap>("aut_global_planner/load_map", std::bind(&GlobalPlanner::LoadMapService, this, std::placeholders::_1, std::placeholders::_2));

  callback_group_nav_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive
  );

  rclcpp::SubscriptionOptions nav_options = rclcpp::SubscriptionOptions();
  nav_options.callback_group = callback_group_nav_;

  nav_modif_subscription_ = this->create_subscription<aut_msgs::msg::NavModif>(
    "aut_local_planner/nav_modif", 10, 
    std::bind(&GlobalPlanner::NavModifCallBack, this, std::placeholders::_1),
    nav_options
  );
}

void GlobalPlanner::LoadMapService(const std::shared_ptr<aut_msgs::srv::LoadMap::Request> request, std::shared_ptr<aut_msgs::srv::LoadMap::Response> response) {
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (goal_is_set_) {
    response->success = false;
    response->message = std::string("Cannot change map while a goal has already been set");
    return;
  }

  std::string path_to_graph = std::string(request->map_directory_path) + std::string("/graph.txt");

  nav_graph_.LoadFile(path_to_graph);
  nav_graph_.Simplify(1.0);
  nav_graph_.TidyGraph();

  initialized_ = true;

  response->success = true;
  response->message = std::string("Map loaded");
  return;
}

rclcpp_action::GoalResponse GlobalPlanner::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const aut_msgs::action::NavigateToGoal::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with goal -> x: %f, y: %f, z: %f", goal->position.x, goal->position.y, goal->position.z);
  (void)uuid;
  std::lock_guard<std::mutex> lock(state_mtx_);
  if (!initialized_ || goal_is_set_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle) {
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&GlobalPlanner::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void GlobalPlanner::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  rclcpp::Rate loop_rate(10);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<aut_msgs::action::NavigateToGoal::Feedback>();
  auto result = std::make_shared<aut_msgs::action::NavigateToGoal::Result>();

  Eigen::Vector3f goal_position(goal->position.x, goal->position.y, goal->position.z);

  state_mtx_.lock();
  int goal_closest_node_idx = nav_graph_.ClosestNode(goal_position);
  Eigen::Matrix4f goal_closest_node_pose = nav_graph_.GetPose(goal_closest_node_idx);
  state_mtx_.unlock();

  if (goal_closest_node_idx < 0 || (goal_closest_node_pose.block<3, 1>(0, 3) - goal_position).squaredNorm() > 2.5f) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted, goal too far from known map");
    return;
  }

  if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted, no localization on map");
    return;
  }

  geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
  Eigen::Matrix4f current_pose = aut_utils::TransformToMatrix(trans);

  state_mtx_.lock();
  int start_closest_node_idx = nav_graph_.ClosestNode(current_pose);
  Eigen::Matrix4f start_closest_node_pose = nav_graph_.GetPose(start_closest_node_idx);
  state_mtx_.unlock();

  if (start_closest_node_idx < 0 || (start_closest_node_pose.block<3, 1>(0, 3) - current_pose.block<3, 1>(0, 3)).squaredNorm() > 1.8f) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted, start too far from known map");
    return;
  }

  std::vector<int> path_idx;
  state_mtx_.lock();
  bool found_path = nav_graph_.AStar(start_closest_node_idx, goal_closest_node_idx, path_idx);
  state_mtx_.unlock();

  if (!found_path) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted, no path to goal");
    return;
  }

  std::vector<Eigen::Vector3f> path_position;
  state_mtx_.lock();
  for (int i = 0; i < (int) path_idx.size(); ++i) {
    path_position.push_back(nav_graph_.GetPose(path_idx[i]).block<3, 1>(0, 3));
  }
  visualization_msgs::msg::MarkerArray current_marker_array = nav_graph_.GetMarkerArrayWithPath(path_idx);
  state_mtx_.unlock();

  goal_is_set_ = true;

  aut_msgs::msg::Nav nav_msg;

  for (int i = 0; i < (int) path_position.size(); ++i) {
    geometry_msgs::msg::Point point;
    point.x = path_position[i](0);
    point.y = path_position[i](1);
    point.z = path_position[i](2);
    nav_msg.positions.push_back(point);
  }

  nav_publisher_->publish(nav_msg);

  while (rclcpp::ok()) {

    if (goal_handle->is_canceling()) {

      aut_msgs::msg::Nav empty_nav_msg;
      nav_publisher_->publish(empty_nav_msg);

      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      goal_is_set_ = false;
      return;
    }

    geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
    Eigen::Matrix4f map_tform_base_link = aut_utils::TransformToMatrix(trans);
    if ((map_tform_base_link.block<3, 1>(0, 3) - goal_position).norm() < 0.2)  {
      aut_msgs::msg::Nav empty_nav_msg;
      nav_publisher_->publish(empty_nav_msg);

      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeded");

      state_mtx_.lock();
      goal_is_set_ = false;
      state_mtx_.unlock();
      return;
    }

    trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
    current_pose = aut_utils::TransformToMatrix(trans);

    state_mtx_.lock();
    if (nav_graph_has_changed_) {

      if (inaccessible_node_in_path_ < 0) {
        aut_msgs::msg::Nav empty_nav_msg;
        nav_publisher_->publish(empty_nav_msg);
        
        result->success = false;
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), "Goal aborted, no path feasible");
        return;
      }

      // Modify nav_graph_
      nav_graph_.RemoveEdge(path_idx[inaccessible_node_in_path_], path_idx[inaccessible_node_in_path_ - 1]);

      start_closest_node_idx = nav_graph_.ClosestNode(current_pose);
      start_closest_node_pose = nav_graph_.GetPose(start_closest_node_idx);
      
      path_idx.clear();
      found_path = nav_graph_.AStar(start_closest_node_idx, goal_closest_node_idx, path_idx);

      if (!found_path) {
        aut_msgs::msg::Nav empty_nav_msg;
        nav_publisher_->publish(empty_nav_msg);

        result->success = false;
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), "Goal aborted, no more path to goal");
        return;
      }

      path_position.clear();
      for (int i = 0; i < (int) path_idx.size(); ++i) {
        path_position.push_back(nav_graph_.GetPose(path_idx[i]).block<3, 1>(0, 3));
      }

      aut_msgs::msg::Nav new_nav_msg;

      for (int i = 0; i < (int) path_position.size(); ++i) {
        geometry_msgs::msg::Point point;
        point.x = path_position[i](0);
        point.y = path_position[i](1);
        point.z = path_position[i](2);
        new_nav_msg.positions.push_back(point);
      }

      nav_publisher_->publish(new_nav_msg);

      current_marker_array = nav_graph_.GetMarkerArrayWithPath(path_idx);

      nav_graph_has_changed_ = false;

      feedback->msg = std::string("Rerouting");
    } else {
      feedback->msg = std::string("Following");
    }
    state_mtx_.unlock();

    int start_path_idx = 0;
    float closest_dist = (current_pose.block<3, 1>(0, 3) - path_position[0]).squaredNorm();
    for (int i = 1; i < (int) path_position.size(); ++i) {
      if ((current_pose.block<3, 1>(0, 3) - path_position[i]).squaredNorm() < closest_dist) {
        closest_dist = (current_pose.block<3, 1>(0, 3) - path_position[i]).squaredNorm();
        start_path_idx = i;
      }
    }

    float path_dist = (current_pose.block<3, 1>(0, 3) - path_position[start_path_idx]).norm();

    for (int i = start_path_idx; i < (int) path_idx.size() - 1; ++i) {
      path_dist += (path_position[i] - path_position[i + 1]).norm();
    }

    feedback->distance = path_dist;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");

    marker_array_publisher_->publish(current_marker_array);

    loop_rate.sleep();
  }
}

void GlobalPlanner::NavModifCallBack(const aut_msgs::msg::NavModif::SharedPtr nav_modif_msg) {
  state_mtx_.lock();
  nav_graph_has_changed_ = true;
  inaccessible_node_in_path_ = nav_modif_msg->inaccessible_node_in_path;
  state_mtx_.unlock();
}

// void GlobalPlanner::timer_callback() {
//   if (goal < 0) {
//     return;
//   }

//   if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
//     return;
//   }

//   geometry_msgs::msg::Transform trans = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero).transform;
//   Eigen::Matrix4f map_tform_base_link = aut_utils::TransformToMatrix(
//     trans
//   );

//   int new_start = nav_graph_.ClosestNode(map_tform_base_link);

//   if (new_start == start) {
//     marker_array_publisher_->publish(last_marker_array);
//     return;
//   }

//   start = new_start;

//   std::vector<int> path;
//   nav_graph_.AStar(start, goal, path);

//   last_marker_array = nav_graph_.GetMarkerArrayWithPath(path);

//   marker_array_publisher_->publish(last_marker_array);
// }

}  // namespace aut_global_planner