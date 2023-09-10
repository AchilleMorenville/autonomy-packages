cmake_minimum_required(VERSION 3.8)
project(aut_tests)

add_compile_options(-Wall -Wextra -Wpedantic -Ofast)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(aut_msgs REQUIRED)


include_directories(
  include
)

add_executable(test_tf2
  src/test_tf2.cc
)
ament_target_dependencies(test_tf2 rclcpp tf2_ros aut_msgs)

install(TARGETS test_tf2
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp tf2_ros aut_msgs)

ament_package()
