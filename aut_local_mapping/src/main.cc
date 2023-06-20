#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aut_local_mapping/local_mapper.h"

int main(int argc, char * argv[]) {
   rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  // rclcpp::executors::SingleThreadedExecutor exec;
  auto local_mapper_node = std::make_shared<aut_local_mapping::LocalMapper>();

  exec.add_node(local_mapper_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Local Mapper Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();
  return 0;
}