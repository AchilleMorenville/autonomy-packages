#include "utils.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("save_map_client");
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client = node->create_client<std_srvs::srv::Trigger>("slam/stop");
  rclcpp::Client<autonomous_interfaces::srv::SlamSaveMap>::SharedPtr save_client = node->create_client<autonomous_interfaces::srv::SlamSaveMap>("slam/save_map");
  
  auto request_stop = std::make_shared<std_srvs::srv::Trigger::Request>();

  while (!stop_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_stop = stop_client->async_send_request(request_stop);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_stop) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopped %d", result_stop.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service stop");
  }

  auto request_save = std::make_shared<autonomous_interfaces::srv::SlamSaveMap::Request>();
  request_save->resolution = 0.01;
  request_save->destination = std::string("/ros2_ws/data/map_go.pcd");

  while (!save_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result_save = save_client->async_send_request(request_save);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result_save) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Stopped %d", result_save.get()->success);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service stop");
  }

  rclcpp::shutdown();
  return 0;
}