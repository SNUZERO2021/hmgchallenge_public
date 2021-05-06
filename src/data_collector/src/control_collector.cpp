#include "ros/ros.h"
#include "ros/console.h"
#include "ros/package.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <fstream>
#include <iostream>
#include <string>

#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace message_filters;

double Ax;
double SteeringWheel;    

void callback(const nav_msgs::Odometry::ConstPtr& odometry, const sensor_msgs::Imu::ConstPtr& imu)
{
    static int count = 0;
    count++;
    //if(count%10!=0)return;
    double az = imu->linear_acceleration.z;
    if(abs(az)>0.05){
        if(abs(az)<0.9) return;
        Ax = 4.0*(double)rand()/(double)RAND_MAX;
        SteeringWheel = 4.0*(double)rand()/(double)RAND_MAX;
        ros::param::set("/Ax", Ax);
        ros::param::set("/SteeringWheel", SteeringWheel);
        return;
    }
        
    geometry_msgs::Quaternion q = odometry->pose.pose.orientation;
    double yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    double x = odometry->pose.pose.position.x;
    double y = odometry->pose.pose.position.y;
    double vx = odometry->twist.twist.linear.x;
    double vy = odometry->twist.twist.linear.y;
    double yawrate = odometry->twist.twist.angular.z;
    double ax = imu->linear_acceleration.x;
    double ay = imu->linear_acceleration.y;
    
    stringstream s;
    s<< ros::package::getPath("data_collector") << "/data/control/" << Ax<<"_"<<SteeringWheel<<".csv";
    ofstream out(s.str(),ios::app);
    //out << mks(x,y,yaw) << mks(vx,vy,yaw) << mks(ax,ay,yaw) << "," << yaw << "," << yawrate 
    //    << "," << Ax << "," << SteeringWheel << "," << 
    //out << x << "," << y << "," << vx << "," << vy << "," << ax << "," << ay << "," << yaw << "," << yawrate << "," << Ax << "," << SteeringWheel << endl;
    //printf("%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",x,y,vx,vy,ax,ay,yaw,yawrate,Ax,SteeringWheel);
}

inline stringstream mks(double kx, double ky, double yaw){
    stringstream s;
    s << "," << kx*cos(yaw)+ky*sin(yaw) << "," << -kx*sin(yaw)+ky*cos(yaw);
    return s;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");
    ros::NodeHandle nh;
    srand (time(NULL));
    Ax = 4.0*(double)rand()/(double)RAND_MAX;
    SteeringWheel = 4.0*(double)rand()/(double)RAND_MAX;
    ros::param::set("/Ax", Ax);
    ros::param::set("/SteeringWheel", SteeringWheel);

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/Odometry", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/Imu", 1);
    typedef sync_policies::ExactTime<nav_msgs::Odometry, sensor_msgs::Imu> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, imu_sub);
    sync.registerCallback(&callback);
    ros::spin();
    return 0;
}
