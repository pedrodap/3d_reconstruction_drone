#include "ros/ros.h"
#include "reconstruct_node.hpp"

void syncAndTfPointCloud::transformPointCloud::callback(const sensor_msgs::PointCloud2ConstPtr& pointCloudLidar, const nav_msgs::OdometryConstPtr& odometryALOAM)
{
    ROS_INFO("Callback %s",odometryALOAM->header.frame_id.c_str());
    typedef pcl::PointCloud<pcl::PointNormal> CloudPointNormalType;
    CloudPointNormalType::Ptr cloud_normals_ptr(new CloudPointNormalType);
    CloudPointNormalType::Ptr mls_points(new CloudPointNormalType);
    CloudPointNormalType::Ptr cloud_normals_ptr_filtered(new CloudPointNormalType);
    //sensor_msgs::PointCloud2 transformedPointCloud2;
    //poseMsgToTF(odometryALOAM->pose.pose, odometryMeasure);
    //tf::Transform odometryMeasureTransform;
//
    ////Create TF matrix
    //double Rx, Ry, Rz, tx, ty, tz;
    //odometryMeasure.getBasis().getEulerYPR(Rz, Ry, Rx);
    //ROS_INFO("%f,%f,%f",Rx,Ry,Rz);
    //tx = odometryMeasure.getOrigin().x();
    //ty = odometryMeasure.getOrigin().y();
    //tz = odometryMeasure.getOrigin().z();
//
    //tf::Quaternion q;
    //q.setRPY(Rz,Ry,Rx);
    //odometryMeasureTransform.setOrigin(tf::Vector3(tx,ty,tz));
    //odometryMeasureTransform.setRotation(q);
//
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFilter (new pcl::PointCloud<pcl::PointXYZ>);
//
//
    //pcl_ros::transformPointCloud("odom", odometryMeasureTransform.inverse(), *pointCloudLidar, transformedPointCloud2);
    pcl::fromROSMsg(*pointCloudLidar, *cloud);
    /*
    *Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    *transformation_matrix (0,3) = tx; transformation_matrix (1,3) = ty;transformation_matrix (2,3) = tz;
    *pcl::transformPointCloud (*cloud, *cloud, transformation_matrix);
    */

    //pcl::fromROSMsg(*pointCloudLidar, *cloud);

   //pcl::VoxelGrid<pcl::PointXYZ> sor;
   //sor.setInputCloud(cloud);
   //sor.setLeafSize(0.1f,0.1f,0.1f);
   //sor.filter(*cloudFilter);
    if (sumCloud -> points.empty())
    {
        *sumCloud = *cloud;
    }
    else
    {
        //icp.setInputSource(cloudFilter);
        //icp.setInputTarget(sumCloud);
        //// Set the max correspondence distance to 5cm (e.g., correspondences with higher
        //// distances will be ignored)
        //icp.setMaxCorrespondenceDistance (0.05);
        //// Set the maximum number of iterations (criterion 1)
        //icp.setMaximumIterations (100);
        //icp.align(*cloudFilter);
        *sumCloud += *cloud;
    }
   countPoint++;
   if(countPoint == 50){
        /* TODO: sub sumcloud with normal cloud and then when feature is proven eliminate this*/
        std::string filename = "/home/alphad/catkin_ws/src/cloud_" + std::to_string(odometryALOAM->header.stamp.sec) + ".pcd";
        pcl::io::savePCDFileASCII (filename, *sumCloud);
        ROS_INFO("PointCloud Written");
        countPoint = 0;
        ///* TODO: Set this into another node for multithreading*/
        ////Normal estimation
        //// Create the normal estimation class, and pass the input dataset to it
        //pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        //ne.setInputCloud (sumCloud);
        //// Create an empty kdtree representation, and pass it to the normal estimation object.
        //// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        //tree->setInputCloud(sumCloud);
        //ne.setSearchMethod (tree);
        //// Output datasets
        //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        //// Use all neighbors in a sphere of radius 3cm
        //ne.setRadiusSearch (0.5);
        //// Compute the features
        //ne.compute (*cloud_normals);  
          // Create a KD-Tree
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

        // Output has the PointNormal type in order to store the normals calculated by MLS
        //pcl::PointCloud<pcl::PointNormal> mls_points;

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
        //pcl::io::savePCDFileASCII (filenameNormal, *cloud_normals);
        pcl::io::savePCDFileASCII (filenameNormal, *mls_points);
        ROS_INFO("Normals estimated"); 
        
        //if (sumCloud_normals -> points.empty())
        //{
        //    *sumCloud_normals = *cloud_normals;
        //}
        //else 
        //{
        //    *sumCloud_normals += *cloud_normals;
        //}

        //pcl::concatenateFields(*sumCloud, *cloud_normals, *cloud_normals_ptr);
        //pcl::concatenateFields(*sumCloud, *mls_points, *cloud_normals_ptr);
        //pcl::StatisticalOutlierRemoval< pcl::PointNormal > stat (new pcl::StatisticalOutlierRemoval< pcl::PointNormal>);
        //stat.setInputCloud(mls_points);
        //stat.setStddevMulThresh(1);
        //stat.setMeanK(50);
        //stat.filter(*cloud_normals_ptr_filtered);
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