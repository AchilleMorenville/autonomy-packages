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
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
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
find_package(dynamicEDT3D REQUIRED)
find_package(octomap REQUIRED)

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

include_directories(${OCTOMAP_INCLUDE_DIRS})
# target_link_libraries(${OCTOMAP_LIBRARIES})
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Cloud.msg" DEPENDENCIES std_msgs sensor_msgs)

# ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(feature_extraction src/feature_extraction.cpp src/utils.cpp)
# target_link_libraries(feature_extraction ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(feature_extraction rclcpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_ros PCL pcl_conversions Eigen OpenCV autonomous_interfaces nav_msgs)
target_link_libraries(feature_extraction ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
# rosidl_target_interfaces(feature_extraction ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(lidar_odometry src/lidar_odometry.cpp src/optimization.cpp src/utils.cpp)
# target_link_libraries(lidar_odometry ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(lidar_odometry rclcpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_ros PCL pcl_conversions Eigen OpenCV autonomous_interfaces nav_msgs)
target_link_libraries(lidar_odometry ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
# rosidl_target_interfaces(lidar_odometry ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(monte_carlo_localization src/monte_carlo_localization.cpp src/utils.cpp)
# target_link_libraries(monte_carlo_localization ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(monte_carlo_localization rclcpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_ros PCL pcl_conversions Eigen OpenCV autonomous_interfaces nav_msgs octomap dynamicEDT3D)
target_link_libraries(monte_carlo_localization ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES} "${cpp_typesupport_target}")
# rosidl_target_interfaces(monte_carlo_localization ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(graph_optimization src/graph_optimization.cpp src/optimization.cpp src/utils.cpp)
# target_link_libraries(graph_optimization ${PCL_LIBRARIES} gtsam ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(graph_optimization rclcpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_ros PCL pcl_conversions Eigen OpenCV autonomous_interfaces GTSAM nav_msgs octomap dynamicEDT3D)
target_link_libraries(graph_optimization gtsam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES} "${cpp_typesupport_target}")
# rosidl_target_interfaces(graph_optimization ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(save_map src/save_map.cpp src/utils.cpp)
# target_link_libraries(graph_optimization ${PCL_LIBRARIES} gtsam ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(save_map rclcpp std_msgs std_srvs sensor_msgs geometry_msgs tf2 tf2_ros PCL pcl_conversions Eigen OpenCV autonomous_interfaces GTSAM nav_msgs)
target_link_libraries(save_map gtsam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} "${cpp_typesupport_target}")
# rosidl_target_interfaces(graph_optimization ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  feature_extraction
  graph_optimization
  lidar_odometry
  monte_carlo_localization
  save_map
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
