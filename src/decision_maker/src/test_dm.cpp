#include <iostream>
#include <sstream>
#include <algorithm>
#include <vector>
#include "tools.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "utils.h"
#include "GlobalMap.h"
#include "cubic_spline_2d.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_dm");

    stringstream segment_info_path;
    segment_info_path << ros::package::getPath("global_map") << "/config/segment_info.txt";
    stringstream segment_map_path;
    segment_map_path << ros::package::getPath("global_map") << "/config/segment_map.txt";
    GlobalMap global_map = GlobalMap(10,0,13500,0,10,15000,24300,21400);
    global_map.loadSegmentInfo(segment_info_path.str());
    global_map.loadSegmentMap(segment_map_path.str());
    vector<CubicSpline2D> cubics = global_map.segment;
    vector<vector<int>> G = vector<vector<int>>(cubics.size());
    for(int i = 0;i<cubics.size();i++){
        G[i] = cubics[i].get_edge();
    }
    vector<string> names = vector<string>(cubics.size());
    for(int i = 0;i<cubics.size();i++){
        names[i] = cubics[i].get_id()[1];
    }

    stringstream waypoint_path;
    waypoint_path << ros::package::getPath("global_map") << "/waypoint/waypoint.csv";
    MissionRecognizer mr;
    mr.loadWaypoint(global_map, cubics, names, waypoint_path.str());

    printf("\n---------------------------------------------------\n");

    stringstream mission_info_path;
    mission_info_path << ros::package::getPath("global_map") << "/mission_config/mission_info.txt";
    mr.loadMissionInfo(mission_info_path.str(), names);
    mr.check_mission(names);
}
