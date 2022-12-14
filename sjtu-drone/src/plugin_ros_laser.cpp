#include "plugin_ros_laser.h"
#include "gazebo/sensors/RaySensor.hh"
namespace gazebo{
void RosLaserPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/){
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_INFO("ROS should be initialized first!");
      return;
    }

    if (!_sensor)
        gzerr << "Invalid sensor pointer.\n";
    this->laser = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    if (!this->laser){
        gzerr << "LaserPlugin equires a LaserSensor.\n";
        return;
    }

    ROS_INFO("The Laser plugin has been loaded!");

    node_handle = new ros::NodeHandle("");
    pub = node_handle->advertise<sensor_msgs::PointCloud2>(topicName, 1, false);
    this->updated_conn = this->laser->ConnectUpdated(boost::bind(&RosLaserPlugin::onNewLaserScans,this));
}

void RosLaserPlugin::onNewLaserScans(){
    //copy data into ros message
    
    laser_msg.header.frame_id = "drone_link";
    laser_msg.header.stamp.sec = this->laser->LastUpdateTime().sec;
    laser_msg.header.stamp.nsec = this->laser->LastUpdateTime().nsec;
    laser_msg.header.seq = this->laser->ParentId();

    laser_msg.angle_increment = 0.523;
    laser_msg.angle_max = 1.570796;
    laser_msg.angle_min = -1.570796;

    laser_msg.time_increment = (1/40) / (100);
    laser_msg.range_max = this->laser->RangeMax();
    laser_msg.range_min = this->laser->RangeMin();

    uint32_t range_count = (laser_msg.angle_max - laser_msg.angle_min)/laser_msg.angle_increment;

    laser_msg.ranges.assign(range_count, this->laser->RangeResolution());
    
    laser_msg.intensities.assign(range_count, 0);

    projector.transformLaserScanToPointCloud("drone_link", laser_msg, cloud, listener);

    pub.publish(cloud);
}


GZ_REGISTER_SENSOR_PLUGIN(RosLaserPlugin)
}