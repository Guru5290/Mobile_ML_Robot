cmake_minimum_required(VERSION 3.5)
project(jie_ware)

# Default to C++14 (or newer if required by your code)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# lidar_loc node
add_executable(lidar_loc src/lidar_loc.cpp)
ament_target_dependencies(lidar_loc
  rclcpp
  cv_bridge
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  OpenCV
)
target_link_libraries(lidar_loc ${OpenCV_LIBRARIES})

# costmap_cleaner node
add_executable(costmap_cleaner src/costmap_cleaner.cpp)
ament_target_dependencies(costmap_cleaner
  rclcpp
  nav_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
)

# lidar_filter_node
add_executable(lidar_filter_node src/lidar_filter_node.cpp)
ament_target_dependencies(lidar_filter_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install
install(TARGETS
  lidar_loc
  costmap_cleaner
  lidar_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export package
ament_package()
