#include <iostream>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Vector3.h>
#include "geometry_msgs/Point32.h"

#include "Lidar.h"

using namespace std;

geometry_msgs::Point32 vector_to_point(tf::Vector3 v){
    geometry_msgs::Point32 p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}


Lidar::Lidar(){

}

Lidar::Lidar(double x, double y, double z, double roll, double pitch, double yaw){
    this->x = x;
    this->y = y;
    this->z = z;
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;

    id.setRPY(0,0,0);

    tf::Quaternion q;
    tf::Vector3 v = tf::Vector3(x,y,z);
    q.setRPY(roll, pitch, yaw);
    lidar_tf = tf::Transform(q,v);

    ROS_WARN("COMPLETE TO INITIALIZE 3D LIDAR");
}

void Lidar::setBoardCaster(ros::Subscriber* subptr){
    sub = subptr;
}

void Lidar::callback(const sensor_msgs::PointCloud::ConstPtr& msg){
    vector<geometry_msgs::Point32> new_points;
    pcl::PointCloud<pcl::PointXYZ> new_clouds;
    // tf::Transform T = odom * lidar_tf;
    tf::Transform T = lidar_tf;

    for(geometry_msgs::Point32 point : msg->points){
        tf::Vector3 p =tf::Vector3(point.x, point.y, point.z);
        tf::Transform m = T * tf::Transform(id, p);
        tf::Vector3 v = m.getOrigin();
        new_points.push_back(vector_to_point(v));
        if(v.x()>30.0 || v.x()<-10.0 || abs(v.y())>20.0 || v.z() > 3.0) continue;
        //v = (odom*m).getOrigin();
        new_clouds.points.push_back(pcl::PointXYZ(v.x(),v.y(),v.z()));
    }
    points = new_points;
    clouds = new_clouds;
}

void Lidar::update(tf::Transform odom){
    this->odom = odom;
}

pcl::PointCloud<pcl::PointXYZ> Lidar::get_clouds(){
    return clouds;
}