#include <unistd.h>
#include<iostream>
#include"ros/ros.h"
#include "nav_msgs/Odometry.h"

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"


#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace syncAndTfPointCloud
{
    using namespace message_filters;
    typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPointCloudOdometry;
    class transformPointCloud
    {
        public:
            transformPointCloud(void ):
                sumCloud(new pcl::PointCloud<pcl::PointXYZ>),
                countPoint(0)
            {
                pointCloudSub.subscribe(n,"/velodyne_points",5);
                odometrySub.subscribe(n,"/aft_mapped_to_init_high_frec",1000);

                syncPointer.reset(new AppSync(syncPointCloudOdometry(10),pointCloudSub, odometrySub));
                syncPointer->registerCallback(boost::bind(&transformPointCloud::callback, this, _1, _2));
            }
        private:
            void callback(const sensor_msgs::PointCloud2ConstPtr& pointCloudLidar,const nav_msgs::OdometryConstPtr& odometryALOAM);

            ros::NodeHandle n;
            ros::Subscriber mSubscriber;
            tf::TransformListener transformListener;
            tf::StampedTransform transform;
            tf::Transform odometryMeasure;

            message_filters::Subscriber<sensor_msgs::PointCloud2> pointCloudSub;
            message_filters::Subscriber<nav_msgs::Odometry> odometrySub;
            typedef message_filters::Synchronizer<syncPointCloudOdometry> AppSync;
            boost::shared_ptr<AppSync> syncPointer;
            pcl::PointCloud<pcl::PointXYZ>::Ptr sumCloud;

            pcl::PCDWriter writer;
            int countPoint;
     };
};