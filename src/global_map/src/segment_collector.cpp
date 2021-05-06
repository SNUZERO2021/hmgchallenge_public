#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <dirent.h>
#include <sys/types.h>
#include <algorithm>
#include <vector>
#include <string>


#include "hellocm_msgs/GPS_Out.h"
#include "nav_msgs/Odometry.h"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include "GlobalMap.h"
#include "cubic_spline_2d.h"
#include "utils.h"
#include "proj.h"

#define foreach BOOST_FOREACH

static void list_dir(const char *path){
    struct dirent *entry;
    DIR *dir = opendir(path);
    if(dir==NULL){
        ROS_ERROR("WRONG BAG FILE PATH");
        return;
    }
    vector<string> bag_files;
    int cnt = 0;
    while((entry = readdir(dir)) != NULL){
        cnt++;
        string s(entry->d_name);
        int len = s.length();
        if(len<4) {
            ROS_ERROR("wrong string : %s", s.c_str());
            continue;
        }
        bag_files.push_back(s.substr(0, len-4));
        cout << cnt << " " << entry->d_name << endl;
    }

    sort(bag_files.begin(), bag_files.end());

    for(string name : bag_files){
        stringstream bag_file;
        bag_file << ros::package::getPath("global_map") << "/bags/" << name << ".bag";
        
        stringstream save_path;
        save_path << ros::package::getPath("global_map") << "/result/" << name << ".txt";
        ofstream out(save_path.str());
        
        rosbag::Bag bag(bag_file.str());
        rosbag::View view(bag, rosbag::TopicQuery("/gps_out"));

        int points = 0;
        BOOST_FOREACH(rosbag::MessageInstance const msg, view){
            hellocm_msgs::GPS_Out::ConstPtr gps = msg.instantiate<hellocm_msgs::GPS_Out>();
            //nav_msgs::Odometry::ConstPtr gps = msg.instantiate<nav_msgs::Odometry>();
            if(gps != NULL){
                double lat = gps->latitude;
                double lon = gps->longitude;
                double z = gps->altitude;

                double x = latlong_to_xy(lat, lon)[0];
                double y = latlong_to_xy(lat, lon)[1];
                //double x = gps->pose.pose.position.x;
                //double y = gps->pose.pose.position.y;
                //double z = gps->pose.pose.position.z;

                out << x << " " << y << " " << z << endl;
                points ++;
            } 
        }

        cout << "Completed to save " << name << " segment : " << save_path.str() << " (# of points : " << points << ")" << endl;

        out.close();
        bag.close();
    }
}

int main(int argc, char **argv){
    ROS_INFO("START SEGMENT COLLECTION");
    ros::init(argc, argv, "segment_collection");

    stringstream bag_file_path_;
    bag_file_path_ <<ros::package::getPath("global_map") << "/bags";

    list_dir(bag_file_path_.str().c_str());
}
