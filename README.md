# 3d_reconstruction_drone

1. Prerequisites

Install prerequisites of https://github.com/HKUST-Aerial-Robotics/A-LOAM

Make aloam work on 20.04
Step 1: Configure the headers to use
#include <opencv2/opencv.hpp> instead of #include <opencv/cv.h>

Step 2: Configure CMakeLists.txt to use
set(CMAKE_CXX_FLAGS "-std=c++14") instead of set(CMAKE_CXX_FLAGS "-std=c++11")

2. Build 3d_reconstruction_drone

cd ~/catkin_ws/src
git clone
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash

3. Run 3d_reconstruction_drone

roslaunch sjtu_drone simple.launch

Another tab

roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch

Another tab

rosrun sjtu_drone drone_waypoints




# Aloam

