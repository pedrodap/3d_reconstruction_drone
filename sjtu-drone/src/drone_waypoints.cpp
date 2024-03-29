#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

ros::Publisher pubTakeOff;
ros::Publisher pubLand;
ros::Publisher pubReset;
ros::Publisher pubCmd;
ros::Publisher pubPosCtrl;
ros::Publisher pubVelMode;

std_msgs::Bool bool_msg;
geometry_msgs::Twist twist_msg;
std_msgs::Empty empty;

bool isVelMode = false;
bool isPosctrl = false;

int wp=0;
double vetorwpx[6]={1,4,1,2,5,0};
double vetorwpy[6]={1,2,3,4,3,0};
unsigned int microsecond = 1000000;

void moveTo(float x, float y, float z) {

  twist_msg.linear.x = x;
  twist_msg.linear.y = y;
  twist_msg.linear.z = z;
  twist_msg.angular.x = 0;
  twist_msg.angular.y = 0;
  twist_msg.angular.z = 0;

  pubCmd.publish(twist_msg);
}

void velMode(bool on) {
  isVelMode = on;

  if (isVelMode == true) {
    ROS_INFO("Switching velocity mode on...");
    bool_msg.data = true;
    pubVelMode.publish(bool_msg);
  } else {
    ROS_INFO("Switching velocity mode off...");
  }
}

void takeOff(void) {
  pubTakeOff.publish(empty);
  ROS_INFO("Starting...");
  usleep(10 * microsecond);
}

void posCtrl(bool on) {
  isPosctrl = on;

  bool_msg.data = on ? 1 : 0;
  pubPosCtrl.publish(bool_msg);
  if (on)
    ROS_INFO("Switching position control on...");
  else
    ROS_INFO("Switching position control off...");
}

void waypointPose(void) {
  for (wp = 0; wp < 6;wp++) {
    ROS_INFO("Flying to (%f, %f, 2) with position control",vetorwpx[wp], vetorwpy[wp]);
    moveTo(vetorwpx[wp], vetorwpy[wp], 2);
    usleep(20 * microsecond);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "drone_waypoints");

  ros::NodeHandle node;
  ros::NodeHandle n;

  pubTakeOff = node.advertise<std_msgs::Empty>("/drone/takeoff", 1, true);
  pubLand = node.advertise<std_msgs::Empty>("/drone/land", 1, true);
  pubReset = node.advertise<std_msgs::Empty>("/drone/reset", 1024);
  pubPosCtrl = node.advertise<std_msgs::Bool>("/drone/posctrl", 1024);
  pubCmd = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pubVelMode = node.advertise<std_msgs::Bool>("/drone/vel_mode", 1024);

  velMode(true);
  takeOff();
  posCtrl(true);
  waypointPose();

  ros::spin();

  return 0;
}