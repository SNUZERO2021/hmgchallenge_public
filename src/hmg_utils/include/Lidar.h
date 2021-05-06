#ifndef LIDAR
#define LIDAR

#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <vector>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "geometry_msgs/Point32.h"
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/point_types.h>

#include "ros/ros.h"

#include "utils.h"

using namespace std;


class Lidar{
    public:
    ros::NodeHandle nh; // nodehandle
    ros::Subscriber* sub; // subscribe point cloud from lidar
    tf::Quaternion id; // identity matrix
    double x,y,z; // lidar relative position for car
    double roll, pitch, yaw; // lidar relative orientation for car
    tf::Transform lidar_tf; // lidar tf for car
    tf::Transform odom; // real-time car tf for global frame
    vector<geometry_msgs::Point32> points; // current point clouds
    pcl::PointCloud<pcl::PointXYZ> clouds;

    Lidar();
    Lidar(double x, double y, double z, double roll, double pitch, double yaw); // initializer
    void setBoardCaster(ros::Subscriber* subptr);
    void callback(const sensor_msgs::PointCloud::ConstPtr& msg); // callback function for subscriber
    void update(tf::Transform odom); // real-time update car tf
    pcl::PointCloud<pcl::PointXYZ> get_clouds();
};

#endif