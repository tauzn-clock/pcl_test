cmake_minimum_required(VERSION 3.0.2)
project(plane_detection)

# Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Find the YAML-CPP library (for YAML support)
find_package(yaml-cpp REQUIRED)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(get_organised_pcd cpp/get_organised_pcd.cpp)
target_link_libraries(get_organised_pcd
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(get_ahc_pcd cpp/get_ahc_pcd.cpp)
target_link_libraries(get_ahc_pcd 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)
