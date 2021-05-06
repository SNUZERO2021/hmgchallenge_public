#ifndef OBJECTSENSOR
#define OBJECTSENSOR

#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <vector>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include "hellocm_msgs/ObjectSensor.h"
#include "hmg_utils/ObjectArray.h"
#include "geometry_msgs/Point32.h"
#include "vehicle_detection/tracker_output.h"

#include "ros/ros.h"

#include "utils.h"

using namespace std;


class ObjectSensor{
    public:
    ros::Subscriber* sub; // subscribe object info from object sensor
    ros::Publisher* pub; // publish object array after decoding
    tf::Quaternion id; // identity matrix
    tf::Vector3 origin;
    double x,y,z; // object sensor relative position for car
    double roll, pitch, yaw; // object sensor relative orientation for car
    tf::Transform sensor_tf; // object sensor tf for car
    tf::Transform odom; // odometry tf
    tf::Vector3 vehicle_vel;
    tf::Vector3 vehicle_ang;
    hellocm_msgs::ObjectSensor data;
    
    ObjectSensor();
    //ObjectSensor(double x, double y, double z, double roll, double pitch, double yaw, string sub_name, string pub_name, ros::NodeHandle &nh); // initializer
    ObjectSensor(double x, double y, double z, double roll, double pitch, double yaw); // initializer
    void setBoardCaster(ros::Subscriber* subptr, ros::Publisher* pubptr);
    void callback(const hellocm_msgs::ObjectSensor::ConstPtr& msg); // callback function for subscriber
    void update(tf::Transform odom, tf::Vector3 vel, tf::Vector3 ang);
};

#endif