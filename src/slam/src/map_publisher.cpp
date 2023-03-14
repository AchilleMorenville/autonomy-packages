#include "utils.hpp"

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node {

public:

  MapPublisher() : Node("map_publisher") {
    map.reset(new pcl::PointCloud<pcl::PointXYZI>());
    map_ds.reset(new pcl::PointCloud<pcl::PointXYZI>());

    std::string map_file("/ros2_ws/data/map_go.pcd");

    if (pcl::io::loadPCDFile<pcl::PointXYZI> (map_file, *map) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Can't load map file !");
    }

    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    voxel_grid.setLeafSize(0.2, 0.2, 0.2);
    voxel_grid.setInputCloud(map);
    voxel_grid.filter(*map_ds);

    pcl::toROSMsg(*map_ds, map_msg);
    map_msg.header.frame_id = "map";

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("slam/map", 10);
    timer_ = this->create_wall_timer(10s, std::bind(&MapPublisher::timer_callback, this));
  }

private:

  pcl::PointCloud<pcl::PointXYZI>::Ptr map;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ds;
  sensor_msgs::msg::PointCloud2 map_msg;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  void timer_callback() {
    map_msg.header.stamp = this->get_clock()->now();
    publisher_->publish(map_msg);
  }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 1, false, std::chrono::milliseconds(500));

  auto MP = std::make_shared<MapPublisher>();
  exec.add_node(MP);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Publisher Started.\033[0m");

  exec.spin();

  rclcpp::shutdown();

  return 0;
}