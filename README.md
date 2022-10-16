# 3d_reconstruction_drone

1. Prerequisites

Install prerequisites of https://github.com/HKUST-Aerial-Robotics/A-LOAM

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
