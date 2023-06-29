#ifndef AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_
#define AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <aut_common/graph.h>
#include <aut_msgs/action/navigate_to_goal.hpp>
#include <aut_msgs/srv/load_map.hpp>
#include <aut_msgs/msg/nav.hpp>
#include <aut_msgs/msg/nav_modif.hpp>

namespace aut_global_planner {

class GlobalPlanner : public rclcpp::Node {

 public:
  explicit GlobalPlanner(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const aut_msgs::action::NavigateToGoal::Goal> goal
  );

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle
  );

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle
  );

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<aut_msgs::action::NavigateToGoal>> goal_handle
  );

  void LoadMapService(const std::shared_ptr<aut_msgs::srv::LoadMap::Request> request, std::shared_ptr<aut_msgs::srv::LoadMap::Response> response);

  void NavModifCallBack(const aut_msgs::msg::NavModif::SharedPtr nav_modif_msg);

  bool initialized_;
  bool goal_is_set_;
  bool nav_graph_has_changed_;
  int inaccessible_node_in_path_;

  aut_common::Graph nav_graph_;

  // tf2 Listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // tf buffer
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  rclcpp::Publisher<aut_msgs::msg::Nav>::SharedPtr nav_publisher_;

  // Action server
  rclcpp_action::Server<aut_msgs::action::NavigateToGoal>::SharedPtr action_server_;

  // Services
  rclcpp::Service<aut_msgs::srv::LoadMap>::SharedPtr load_map_service_;

  // Subscription
  rclcpp::Subscription<aut_msgs::msg::NavModif>::SharedPtr nav_modif_subscription_;

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr callback_group_nav_;

  // Mutex
  std::mutex state_mtx_;

  // void timer_callback();
};

}  // namespace aut_global_planner

#endif  // AUT_GLOBAL_PLANNER_GLOBAL_PLANNER_H_