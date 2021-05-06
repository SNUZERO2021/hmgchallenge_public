#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include <queue>
#include <unistd.h>


#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "hellocm_msgs/ObjectSensor.h"
#include "hellocm_msgs/ObjectSensorObj.h"
#include "hellocm_msgs/ObservPoint.h"


#include "opencv2/opencv.hpp"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"


#include "Lidar.h"
#include "utils.h"


using namespace std;
using namespace cv;

typedef pair<double,double> pdd;
typedef pair<int, int> pii;

const double ESP =  1e-2;

struct pos{
    public:
    double x,y,z,roll,pitch, yaw;
    pos() : pos(0,0,0,0,0,0){}
    pos(double x1, double y1, double z1, double r1, double p1, double yy1) : x(x1), y(y1), z(z1), roll(r1), pitch(p1), yaw(yy1){}
};




class DataCollector{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Subscriber sub_object_;
    ros::Subscriber sub_lidar_1_;
    ros::Subscriber sub_lidar_2_;
    //ros::Subscriber sub_lidar_3_;
    vector<Lidar*> lidars;
    Lidar lidar1;
    Lidar lidar2;
    //Lidar lidar3;
    pos object_sensor;
    tf::Quaternion id; // identity matrix
    tf::Vector3 origin;
    tf::Transform sensor_tf; // object sensor tf for car
    tf::Transform odom; // odometry tf
    tf::Vector3 vehicle_vel;
    tf::Vector3 vehicle_ang;
    double resolution = 0.1;
    double h_threshold = 3.0;
    double a1,b1,c1,a2,b2,c2;
    int index;
    int scenario;


    public:
    DataCollector(){
        sub_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 1, &DataCollector::callback, this);
        sub_object_ = nh_.subscribe<hellocm_msgs::ObjectSensor  >("/Object_Sensor_front", 1, &DataCollector::callback_object, this);

        ROS_WARN("CHECK SCENARIO NUMBER!!!");
        ROS_WARN("CHECK SCENARIO NUMBER!!!");
        ROS_WARN("CHECK SCENARIO NUMBER!!!");
        /* please change this value before starting data collection */
        scenario = 6;

        /* set const variable */
        id.setRPY(0,0,0);
        origin = tf::Vector3(0,0,0);


        /* initialize object sensor position */
        object_sensor = pos(-15.0,0,0,0,0,0);
        tf::Quaternion q;
        tf::Vector3 v = tf::Vector3(object_sensor.x,object_sensor.y,object_sensor.z);
        q.setRPY(object_sensor.roll, object_sensor.pitch, object_sensor.yaw);
        sensor_tf = tf::Transform(q,v);

        /* initialize lidar list */
        lidar1 = Lidar(2.0, 0.0, 2.0, 0.0, 0.0, 90.0*M_PI/180.0);
        lidar2 = Lidar(2.5, 0.0, 2.0, 0.0, 0.0, 0.0);
        //lidar3 = Lidar(2.0, -0.8, 1.7, 0.0, 0.0, 0.0);
        sub_lidar_1_ = nh_.subscribe<sensor_msgs::PointCloud>("/pointcloud/vlp", 1, &Lidar::callback, &lidar1);
        sub_lidar_2_ = nh_.subscribe<sensor_msgs::PointCloud>("/pointcloud/os1", 1, &Lidar::callback, &lidar2);
        //sub_lidar_3_ = nh_.subscribe<sensor_msgs::PointCloud>("/pointcloud/vlp1", 1, &Lidar::callback, &lidar3);
        lidars.push_back(&lidar1);
        lidars.push_back(&lidar2);
        //lidars.push_back(&lidar3);


        /* we need to add inserting  lidars info part */
    }

    string get_string(int n, int d){
        if(n>pow(10,d)) {
            ROS_ERROR("N is too large... can't change into string");
            return "";
        }
        if(n==0) return "0000";
        int digit = int(log10(n));
        string rt = "";
        for(int i=0;i<d-1-digit;i++) rt += "0";
        rt += to_string(n);
        return rt;
    }

    pii XY_To_Pixel(pdd p){
        pii rt;
        rt.first = int(a1 * p.first + b1 * p.second + c1);
        rt.second = int(a2 * p.first + b2 * p.second + c2);
        return rt;
    }

    void callback(const nav_msgs::Odometry::ConstPtr& msg){
        clock_t begin = clock();
        /* update odom tf */
        geometry_msgs::Quaternion q = msg->pose.pose.orientation;
        geometry_msgs::Point p = msg->pose.pose.position;
        odom = tf::Transform(tf::Quaternion(q.x,q.y,q.z,q.w),tf::Vector3(p.x,p.y,p.z));
        vehicle_vel = tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        vehicle_ang = tf::Vector3(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
        

        //force data generation frequency 10Hz
        double t = msg->header.stamp.toSec();
        if(t*10-int(round((t*10)))>ESP) return;
        int time = int(msg->header.stamp.toSec()*10);
        
        /* set save path for Odometry */
        stringstream save_path_odom;
        save_path_odom << ros::package::getPath("data_collector") << "/data/control/" << get_string(scenario,2) << "/Odom/Odom_Scenario" << get_string(scenario,2) << "_" << get_string(time, 4) << ".txt";
        ofstream out_odom(save_path_odom.str());

        /* save odometry info */
        q = msg->pose.pose.orientation;
        p = msg->pose.pose.position;
        out_odom << to_string(p.x) << " " << to_string(p.y) << " " << to_string(p.z) << endl; // vehicle position
        out_odom << to_string(q.x) << " " << to_string(q.y) << " " << to_string(q.z) << " " << to_string(q.w) << endl; // vehicle orientation
        out_odom << to_string(msg->twist.twist.linear.x) << " " << to_string(msg->twist.twist.linear.y) << " " << to_string(msg->twist.twist.linear.z) << endl; // vehicle linear velocity
        out_odom << to_string(msg->twist.twist.angular.x) << " " << to_string(msg->twist.twist.angular.y) << " " << to_string(msg->twist.twist.angular.z) << endl; // vehicle angular velocity

        ROS_WARN("save odom data : [path : %s ]", save_path_odom.str().c_str());

        /* need to sleep */
        for(Lidar* lidar : lidars) lidar->update(odom);
        sleep(0.01);

        /* set save path for Odometry */
        stringstream save_path_lidar;
        save_path_lidar << ros::package::getPath("data_collector") << "/data/Scenario" << get_string(scenario,2) << "/Lidar/Lidar_Scenario" << get_string(scenario,2) << "_" << get_string(time, 4) << ".txt";
        ofstream out_lidar(save_path_lidar.str());


        /* save lidar info */
        int cnt = 0;
        ROS_WARN("# OF POINT CLOUD OF LIDAR 1 :  %d", lidar1.points.size());
        ROS_WARN("# OF POINT CLOUD OF LIDAR 1 :  %d", lidars[0]->points.size());
        for(Lidar* lidar : lidars) cnt += lidar->points.size();
        ROS_WARN("# OF POINT CLOUD : %d",cnt);
        out_lidar << to_string(cnt) << endl;
        for(Lidar* lidar : lidars){
            for(geometry_msgs::Point32 point : lidar->points) out_lidar <<  to_string(point.x) << " " <<  to_string(point.y) << " " <<  to_string(point.z) << endl;
        }

        ROS_WARN("save lidar data : [path : %s ]", save_path_lidar.str().c_str());

        /* close file */
        out_odom.close();
        out_lidar.close();
        clock_t end = clock();
        ROS_WARN("TIME : %lf", double(end-begin)/CLOCKS_PER_SEC);

    }

    void callback_object(const hellocm_msgs::ObjectSensor::ConstPtr& msg){
        clock_t begin = clock();
        ROS_WARN("CALLBACK OBJECT SENSOR INFO");
        //force data generation frequency 10Hz
        double t = msg->header.stamp.toSec();
        if(t*10-int(round(t*10))>ESP) return;
        int time = int(msg->header.stamp.toSec()*10);
        
        //sleep(0.01);

        /* set save path for Object */
        stringstream save_path_object;
        save_path_object << ros::package::getPath("data_collector") << "/data/Scenario" << get_string(scenario,2) << "/Object/Object_Scenario" << get_string(scenario,2) << "_" << get_string(time, 4) << ".txt";
        ofstream out_object(save_path_object.str());

        /* save object info */
        int n = msg->Objects.size();
        ROS_WARN("# OF OBJECTS : %d", n);
        out_object << to_string(n) << endl;
        for(hellocm_msgs::ObjectSensorObj object : msg->Objects){
            // test its validation
            //if(object.Name[0] != 'T') continue;
            out_object << object.Name << " " << object.Name[0] << " " << to_string(object.l) << " " << to_string(object.w) << " " << to_string(object.h) << endl;
            
            /* calculate object relative position */
            tf::Vector3 ref_position = tf::Vector3(object.RefPnt.ds[0],object.RefPnt.ds[1],object.RefPnt.ds[2]);
            tf::Quaternion ref_orientation;
            ref_orientation.setRPY(object.RefPnt.r_zyx[0],object.RefPnt.r_zyx[1],object.RefPnt.r_zyx[2]);
            tf::Transform ref_T = sensor_tf * tf::Transform(ref_orientation, ref_position);
            tf::Vector3 obj_position = tf::Vector3(object.l/2,0,0);
            tf::Transform obj_T = ref_T * tf::Transform(id, obj_position);
            tf::Vector3 position = obj_T.getOrigin();

            out_object << to_string(position.x()) << " " << to_string(position.y()) << " " << to_string(position.z()) << endl;
            out_object << to_string(object.RefPnt.r_zyx[0]) << " " << to_string(object.RefPnt.r_zyx[1]) << " " << to_string(object.RefPnt.r_zyx[2]) << endl;

            /* calculate object relative velocity */
            tf::Vector3 velocity_position = tf::Vector3(object.RefPnt.dv[0],object.RefPnt.dv[1],object.RefPnt.dv[2]);
            velocity_position -= (ref_position+sensor_tf.getOrigin()-position).cross(vehicle_ang);

            out_object << to_string(velocity_position.x()) << " " << to_string(velocity_position.y()) << " " << to_string(velocity_position.z()) << endl;
            ROS_WARN("SAVED OBJECT NAME : %s", object.Name.c_str());
        }

        /* close file */
        out_object.close();
        ROS_WARN("save object data : [path : %s ]", save_path_object.str().c_str());


        clock_t end = clock();
        ROS_WARN("TIME : %lf", double(end-begin)/CLOCKS_PER_SEC);
    }

};


int main(int argc, char **argv){
    ros::init(argc, argv, "data_collector");
    DataCollector data_collector;

    ros::spin();
}
