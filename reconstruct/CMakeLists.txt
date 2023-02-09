cmake_minimum_required(VERSION 3.0.2)
project(reconstruct)
## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)

find_package (PCL REQUIRED) 
#set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.13 /usr/local/include/eigen3/)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  nav_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES reconstruct
#  CATKIN_DEPENDS message_filters pcl_conversions pcl_ros pcl roscpp sensor_msgs nav_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(${PROJECT_NAME}_node src/reconstruct_node.cpp)


target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)


