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

    pcl::fromROSMsg(*pointCloudLidar, *cloud);

    if (sumCloud -> points.empty())
    {
        *sumCloud = *cloud;
    }
    else
    {
        *sumCloud += *cloud;
    }

   countPoint++;
   if(countPoint == 50){
        /* TODO: sub sumcloud with normal cloud and then when feature is proven eliminate this*/
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
        //reset sumCloud to last value, will implement concatenation of meshes once end product is good
        *sumCloud = *cloudFilter;
   }
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_point");
    syncAndTfPointCloud::transformPointCloud transformPointCLoud;
    ros::spin();
}