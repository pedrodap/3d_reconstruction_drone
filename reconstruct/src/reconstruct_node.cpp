#include "ros/ros.h"
#include "reconstruct_node.hpp"

void syncAndTfPointCloud::transformPointCloud::callback(const sensor_msgs::PointCloud2ConstPtr& pointCloudLidar, const nav_msgs::OdometryConstPtr& odometryALOAM)
{
    ROS_INFO("Callback %s",odometryALOAM->header.frame_id.c_str());
    typedef pcl::PointCloud<pcl::PointNormal> CloudPointNormalType;
    CloudPointNormalType::Ptr cloud_normals_ptr(new CloudPointNormalType);
    CloudPointNormalType::Ptr mls_points(new CloudPointNormalType);
    CloudPointNormalType::Ptr cloud_normals_ptr_filtered(new CloudPointNormalType);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 transformedPointCloud2;
    poseMsgToTF(odometryALOAM->pose.pose, odometryMeasure);
    tf::Transform odometryMeasureTransform;

    //Create TF matrix
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
    pcl_ros::transformPointCloud("odom", odometryMeasureTransform.inverse(), *pointCloudLidar, transformedPointCloud2);
    pcl::fromROSMsg(transformedPointCloud2, *cloud);
    
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f,0.1f,0.1f);
    sor.filter(*cloudFilter);

    if (sumCloud -> points.empty())
    {
        *sumCloud = *cloudFilter;
    }
    else
    {
        icp.setInputSource(cloudFilter);
        icp.setInputTarget(sumCloud);
        // Set the max correspondence distance to 5cm (e.g., correspondences with higher
        // distances will be ignored)
        icp.setMaxCorrespondenceDistance (0.05);
        // Set the maximum number of iterations (criterion 1)
        icp.setMaximumIterations (100);
        icp.align(*cloudFilter);
        *sumCloud += *cloudFilter;
    }

   countPoint++;
   if(countPoint == 50){
        std::string filename = "/home/alphad/catkin_ws/src/cloud_" + std::to_string(odometryALOAM->header.stamp.sec) + ".pcd";
        pcl::io::savePCDFileASCII (filename, *sumCloud);
        ROS_INFO("PointCloud Written");
        countPoint = 0;
        // Create a KD-Tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        // Init object (second point type is for the normals, even if unused)
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
        
        mls.setComputeNormals (true);
        // Set parameters
        mls.setInputCloud (sumCloud);
        mls.setPolynomialOrder (2);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (0.05);

        // Reconstruct
        mls.process (*mls_points);
        std::string filenameNormal = "/home/alphad/catkin_ws/src/normal_" + std::to_string(odometryALOAM->header.stamp.sec) + ".pcd";

        pcl::io::savePCDFileASCII (filenameNormal, *mls_points);
        ROS_INFO("Normals estimated"); 
        pcl::search::KdTree<pcl::PointNormal>::Ptr treeNormal (new pcl::search::KdTree<pcl::PointNormal> ());
        treeNormal->setInputCloud(mls_points);
        pcl::Poisson<pcl::PointNormal> poisson;
        poisson.setDepth (12);
        poisson.setInputCloud(mls_points);
        poisson.setSearchMethod(treeNormal);
        pcl::PolygonMesh mesh_poisson;
        poisson.reconstruct (mesh_poisson);
        std::string meshfilename = "/home/alphad/catkin_ws/src/mesh_" + std::to_string(odometryALOAM->header.stamp.sec) + ".vtk";
        pcl::io::saveVTKFile(meshfilename, mesh_poisson);
        ROS_INFO("Mesh created"); 
        //reset sumCloud to last value
        *sumCloud = *cloudFilter;
   }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_point");
    syncAndTfPointCloud::transformPointCloud transformPointCLoud;
    ros::spin();
}