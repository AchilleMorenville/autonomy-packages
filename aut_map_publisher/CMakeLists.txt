cmake_minimum_required(VERSION 3.8)
project(aut_map_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(aut_common REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(map_publisher
  src/main.cc
)
ament_target_dependencies(map_publisher rclcpp PCL pcl_conversions sensor_msgs aut_common visualization_msgs Eigen3)
target_link_libraries(map_publisher ${PCL_LIBRARIES} ${Eigen3_LIBRARIES})

install(TARGETS
  map_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
