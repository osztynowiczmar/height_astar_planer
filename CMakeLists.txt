cmake_minimum_required(VERSION 3.8)
project(height_astar_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# A* planner
add_executable(astar_planner_node
  src/astar_planner_node.cpp
)

target_compile_features(astar_planner_node PUBLIC cxx_std_17)

ament_target_dependencies(astar_planner_node
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

# Map 1: multi hills and craters
add_executable(multi_hills_craters_map_publisher_node
  src/multi_hills_craters_map_publisher_node.cpp
)

target_compile_features(multi_hills_craters_map_publisher_node PUBLIC cxx_std_17)

ament_target_dependencies(multi_hills_craters_map_publisher_node
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

# Map 2: hills, craters and labyrinth walls
add_executable(labyrinth_map_publisher_node
  src/labyrinth_map_publisher_node.cpp
)

target_compile_features(labyrinth_map_publisher_node PUBLIC cxx_std_17)

ament_target_dependencies(labyrinth_map_publisher_node
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
)

install(TARGETS
  astar_planner_node
  multi_hills_craters_map_publisher_node
  labyrinth_map_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()