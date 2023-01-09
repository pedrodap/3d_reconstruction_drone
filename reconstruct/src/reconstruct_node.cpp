#include "ros/ros.h"
#include "reconstruct_node.hpp"

void syncAndTfPointCloud::transformPointCloud::callback(const sensor_msgs::PointCloud2ConstPtr& pointCloudLidar, const nav_msgs::OdometryConstPtr& odometryALOAM)
{
   ROS_INFO("Callback %s",odometryALOAM->header.frame_id.c_str());

   sensor_msgs::PointCloud2 transformedPointCloud2;
   poseMsgToTF(odometryALOAM->pose.pose, odometryMeasure);
   tf::Transform odometryMeasureTransform;

   //坐标轴变化
   double Rx, Ry, Rz, tx, ty, tz;
   odometryMeasure.getBasis().getEulerYPR(Rz, Ry, Rx);
   ROS_INFO("%f,%f,%f",Rx,Ry,Rz);
   tx = odometryMeasure.getOrigin().x();
   ty = odometryMeasure.getOrigin().y();
   tz = odometryMeasure.getOrigin().z();

   tf::Quaternion q;
   q.setRPY(Rz,Ry,Rx);
   odometryMeasureTransform.setOrigin(tf::Vector3(tx,ty,tz));
   odometryMeasureTransform.setRotation(q);

   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter (new pcl::PointCloud<pcl::PointXYZ>);


   pcl_ros::transformPointCloud("odom", odometryMeasureTransform.inverse(), *pointCloudLidar, transformedPointCloud2);
   pcl::fromROSMsg(transformedPointCloud2, *cloud);
   /*
    *Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    *transformation_matrix (0,3) = tx; transformation_matrix (1,3) = ty;transformation_matrix (2,3) = tz;
    *pcl::transformPointCloud (*cloud, *cloud, transformation_matrix);
   */

   pcl::VoxelGrid<pcl::PointXYZ> sor;
   sor.setInputCloud(cloud);
   sor.setLeafSize(0.01f,0.01f,0.01f);
   sor.filter(*cloudFilter);
   *sumCloud += *cloudFilter;

   if(countPoint == 6)
       writer.write("sum.pcd",*sumCloud);

    countPoint++;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_point");
    syncAndTfPointCloud::transformPointCloud transformPointCLoud;
    ros::spin();
}
