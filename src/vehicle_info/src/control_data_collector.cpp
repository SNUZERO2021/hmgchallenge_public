#include "ros/ros.h"
#include "ros/package.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <iostream>
#include <fstream>
#include <string>
#include "hellocm_msgs/VehicleInfo.h"
#include "nav_msgs/Odometry.h"
#include "utils.h"


using namespace std;
using namespace message_filters;

class ControlData{
 private:
    ros::NodeHandle nh_;
    // ros::Subscriber sub_vehicleinfo_;
    // ros::Subscriber sub_filtered_vehicleinfo;
    // ros::Subscriber sub_odometry;
    message_filters::Subscriber<hellocm_msgs::VehicleInfo> fsub_vehicleinfo_;
    message_filters::Subscriber<hellocm_msgs::VehicleInfo> fsub_filtered_vehicleinfo_;
    message_filters::Subscriber<nav_msgs::Odometry> fsub_odometry_;
    typedef sync_policies::ExactTime<hellocm_msgs::VehicleInfo, hellocm_msgs::VehicleInfo, nav_msgs::Odometry> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync;
    stringstream data_path;
    ofstream out;

 public:
    ControlData():sync(MySyncPolicy(10), fsub_vehicleinfo_, fsub_filtered_vehicleinfo_, fsub_odometry_){
        // sub_vehicleinfo_ = nh_.subscribe("/vehicleinfo",1,&ControlData::Callback_VehicleInfo,this);
        // sub_filtered_vehicleinfo = nh_.subscribe("/filtered_vehicleinfo",1,&ControlData::Callback_FilteredVehicleInfo,this);
        // sub_odometry = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 1, &ControlData::Callback_Odometry, this);

        fsub_vehicleinfo_.subscribe(nh_, "/vehicleinfo", 1);
        fsub_filtered_vehicleinfo_.subscribe(nh_, "/filtered_vehicleinfo", 1);
        fsub_odometry_.subscribe(nh_, "/Odometry", 1);
        sync.registerCallback(&ControlData::callback_sync,this);

        data_path<< ros::package::getPath("vehicle_info") << "/data/" << "data.txt";
    }

    void callback_sync(const hellocm_msgs::VehicleInfo::ConstPtr& vehicleinfo, const hellocm_msgs::VehicleInfo::ConstPtr& filtered_vehicleinfo, const nav_msgs::Odometry::ConstPtr& odometry){
        out = ofstream(data_path.str(), std::ios::app);

        Callback_Odometry(odometry);
        Callback_VehicleInfo(*vehicleinfo);
        Callback_FilteredVehicleInfo(*filtered_vehicleinfo);
        out << endl;

        out.close();
    }

    void Callback_Odometry(const nav_msgs::Odometry::ConstPtr& msg){
        // stringstream data_path;
        // string filename="test";
        // data_path<< ros::package::getPath("vehicle_info") << "/data/" << filename<< ".txt";
        // ofstream out(data_path.str());

        geometry_msgs::Quaternion q = msg->pose.pose.orientation;
        geometry_msgs::Point p = msg->pose.pose.position;
        out << to_string(p.x) << "|" << to_string(p.y) << "|" << to_string(p.z) <<"|"; // vehicle position
        out << to_string(q.x) << "|" << to_string(q.y) << "|" << to_string(q.z) << "|" << to_string(q.w)<<"|"; // vehicle orientation
        // out << to_string(msg->twist.twist.linear.x) << "|" << to_string(msg->twist.twist.linear.y) << "|" << to_string(msg->twist.twist.linear.z)<<"|"; // vehicle linear velocity
        out << to_string(msg->twist.twist.angular.x) << "|" << to_string(msg->twist.twist.angular.y) << "|" << to_string(msg->twist.twist.angular.z)<<"|"; // vehicle angular velocity
        // out.close();
        //ROS_WARN("save data : [path : %s ]", save_path_odom.str().c_str());
    
    }

    void Callback_VehicleInfo(const hellocm_msgs::VehicleInfo& msg){
        // stringstream data_path;
        // data_path<< ros::package::getPath("vehicle_info") << "/data/" << "data.txt";
        // ofstream out(data_path.str(), std::ios::app);

        out << to_string(msg.v[0]) << "|" << to_string(msg.v[1]) << "|" << to_string(msg.a[0]) << "|" << to_string(msg.a[1]) << "|" ;
        out << to_string(msg.yaw) << "|" << to_string(msg.yawrate) << "|" << to_string(msg.yawacc)<<"|";
        ROS_WARN("yawv %lf",msg.yawrate);
        // out << to_string(msg.steer) << "|" << to_string(msg.gas) << "|" << to_string(msg.brake)<<"|";
        // out.close();
    }

    void Callback_FilteredVehicleInfo(const hellocm_msgs::VehicleInfo& msg){
        // stringstream data_path;
        // data_path<< ros::package::getPath("vehicle_info") << "/data/" << "data.txt";
        double Ax;
        double SteeringWheel;
        // ofstream out(data_path.str(), std::ios::app);

        ros::param::get("Ax",Ax);
        ros::param::get("SteeringWheel",SteeringWheel);

        out << to_string(msg.x[0]) << "|" << to_string(msg.x[1]) << "|" << to_string(msg.v[0]) << "|" << to_string(msg.v[1]) << "|" << to_string(msg.a[0]) << "|" << to_string(msg.a[1]) << "|" ;
        out << to_string(msg.yaw) << "|" << to_string(msg.yawrate) << "|" << to_string(msg.yawacc)<<"|";
        out << to_string(msg.steer) << "|" << to_string(msg.gas) << "|" << to_string(msg.brake)<<"|" << Ax <<"|" <<SteeringWheel<<"|" ;//<<endl;
        ROS_WARN("fyawv %lf",msg.yawrate);
        // out.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_data_collector");
    
    ControlData control_data;
    
    ros::spin();
    return 0;
}

