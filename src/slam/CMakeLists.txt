cmake_minimum_required(VERSION 3.8)
project(slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(autonomous_interfaces)
find_package(GTSAM REQUIRED)

# include_directories(
#   ${PCL_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${GTSAM_INCLUDE_DIR}
#   include/slam
# )

# link_directories(
#   ${OpenCV_LIBRARY_DIRS}
#   ${PCL_LIBRARY_DIRS}
# )

include_directories(
  include/slam
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Cloud.msg" DEPENDENCIES std_msgs sensor_msgs)

# ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(feature_extraction src/feature_extraction.cpp src/utils.cpp)
# target_link_libraries(feature_extraction ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(feature_extraction rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros PCL pcl_conversions Eigen OpenCV autonomous_interfaces)
target_link_libraries(feature_extraction ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
# rosidl_target_interfaces(feature_extraction ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(graph_optimization src/graph_optimization.cpp src/optimization.cpp src/utils.cpp)
# target_link_libraries(graph_optimization ${PCL_LIBRARIES} gtsam ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(graph_optimization rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros PCL pcl_conversions Eigen OpenCV autonomous_interfaces GTSAM)
target_link_libraries(graph_optimization gtsam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
# rosidl_target_interfaces(graph_optimization ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  feature_extraction
  graph_optimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
