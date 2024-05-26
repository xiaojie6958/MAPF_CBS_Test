cmake_minimum_required(VERSION 3.0.2)
project(mapf_ros)


# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS cmake_modules
             roscpp
             rospy
             std_msgs
             nav_msgs
             geometry_msgs
             sensor_msgs
             mapf_msgs
             costmap_2d
             tf2_geometry_msgs
             tf2_ros
             move_base_msgs
             actionlib
             roslib
             pluginlib)

find_package(TinyXML REQUIRED)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  nav_msgs
  mapf_msgs
  costmap_2d
  tf2_geometry_msgs
  tf2_ros
  move_base_msgs
  actionlib
  roslib
  pluginlib
  DEPENDS system_lib TinyXML)

include_directories(include ${catkin_INCLUDE_DIRS} ${TinyXML_INCLUDE_DIRS})

# my_cbs
add_executable(my_cbs_ros include/mapf_ros/my_cbs/path_finder_algorithm/dijkstra.cpp 
                          include/mapf_ros/my_cbs/parser.cpp 
                          src/my_cbs_ros.cpp)
add_dependencies(my_cbs_ros ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(my_cbs_ros ${catkin_LIBRARIES} ${TinyXML_LIBRARIES})