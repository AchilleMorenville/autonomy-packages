cmake_minimum_required(VERSION 3.8)
project(aut_local_mapping)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aut_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(aut_utils REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(
  include
)

add_library(voxel_grid SHARED
  src/voxel_grid.cc
)
ament_target_dependencies(voxel_grid rclcpp aut_msgs Eigen3 PCL aut_utils)
target_link_libraries(voxel_grid ${PCL_LIBRARIES}) 

add_library(local_mapper SHARED
  src/local_mapper.cc
)
ament_target_dependencies(local_mapper rclcpp aut_msgs Eigen3 PCL pcl_conversions aut_utils visualization_msgs)
target_link_libraries(local_mapper voxel_grid ${PCL_LIBRARIES}) 

add_executable(local_mapping
  src/main.cc
)
ament_target_dependencies(local_mapping rclcpp aut_msgs PCL Eigen3 pcl_conversions aut_utils visualization_msgs)
target_link_libraries(local_mapping local_mapper voxel_grid ${PCL_LIBRARIES})

install(TARGETS local_mapper voxel_grid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS local_mapping
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(local_mapper voxel_grid)
ament_export_dependencies(rclcpp aut_msgs PCL Eigen3 pcl_conversions aut_utils visualization_msgs)

ament_package()