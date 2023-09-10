#include <chrono>
#include <memory>
#include <thread>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/common/io.h>

#include <Eigen/Core>

#include <sensor_msgs/msg/point_cloud2.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <aut_common/graph.h>

class MapPublisher : public rclcpp::Node {

 public:
  explicit MapPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("monte_carlo_localization", options) {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("aut_map_publisher/map", 10);
    marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("aut_map_publisher/graph", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapPublisher::timer_callback, this));
    std::string map_path("/ros2_ws/src/data/home-map/map.pcd");

    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile<pcl::PointXYZI> (map_path, *map_cloud);

    pcl::toROSMsg(*map_cloud, map_msg);
    map_msg.header.frame_id = "map";

    std::string graph_path("/ros2_ws/src/data/home-map/graph.txt");
    nav_graph_.LoadFile(graph_path);

    nav_graph_.Simplify(0.5);
    nav_graph_.TidyGraph();

    marker_array_msg = nav_graph_.GetMarkerArray();
  }

 private:
  void timer_callback() {
    publisher_->publish(map_msg);
    marker_array_publisher_->publish(marker_array_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
  sensor_msgs::msg::PointCloud2 map_msg;
  visualization_msgs::msg::MarkerArray marker_array_msg;
  aut_common::Graph nav_graph_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500)
  );

  // rclcpp::executors::SingleThreadedExecutor exec;
  auto map_publisher_node = std::make_shared<MapPublisher>();
  // auto lidar_odometry_node = std::make_shared<aut_lidar_odometry::LidarOdometry>();
  
  exec.add_node(map_publisher_node);
  // exec.add_node(lidar_odometry_node);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Publisher Started.\033[0m");
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Lidar Odometry Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();
  return 0;
}