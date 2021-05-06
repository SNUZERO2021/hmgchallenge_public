#include "hmg_utils/Light.h"
#include "hellocm_msgs/VehicleInfo.h"
#include "hmg_utils/Targets.h"
#include "utils.h"
#include "GlobalMap.h"
#include "cubic_spline_2d.h"
#include <iostream>
#include "ros/ros.h"
#include "ros/package.h"

using namespace std;

enum taskState{
	DRIVING_SECTION,
	INTERSECTION_STRAIGHT,
	INTERSECTION_LEFT,
	INTERSECTION_RIGHT,
};

enum motionState{
    DRIVE,
    LANE_CHANGE,
    CAUTION,
    STOP,
};

bool debug;
bool time_debug;
double l_range;
double check_l_range;
double dl_range;
double keep_light_time;

class LightControlNode{
    public:
        ros::NodeHandle nh;
        ros::Subscriber vehicleinfo_sub;
        ros::Subscriber targets_sub;

        // targets
        int current_task;
        int next_task;
        int motion;
        int changing_dir;
        bool targets_intersection;

        // vehicle info
        double x;
        double y;
        double vx;
        double vy;

        // global map
        GlobalMap global_map;			// to load Global map
		vector<CubicSpline2D> cubics;	// cubic spline segments
		vector<vector<int>> G;			// edge information

        int flag_location;
        int flag_targets;
        int flag_dl;
        int flag_turnon;
        double last_light_time;

        // // subscribing info
        // int current_task;
        // int next_task;
        // int current_segment;
        // // state machine
        // int state;                      // 0 : near segment, 1 : lane change, 2 : intersection
        // int flag;                       // 1 : left, -1 : right (valid when state = 1) 
        // int nearest_segment;
        // int real_turnon_flag;
        // double last_light_time;

        LightControlNode(){
            ROS_WARN("[light_control] init");
            stringstream segment_info_path;
			segment_info_path << ros::package::getPath("global_map") << "/config/segment_info.txt";
            stringstream segment_map_path;
    		segment_map_path << ros::package::getPath("global_map") << "/config/segment_map.txt";
			stringstream intersection_map_path;
    		intersection_map_path << ros::package::getPath("global_map") << "/config/intersection_map.txt";
			global_map = GlobalMap(10,0,13500,0,10,15000,24300,21400);
    		global_map.loadSegmentInfo(segment_info_path.str());
			global_map.loadSegmentMap(segment_map_path.str());
            global_map.loadIntersectionMap(intersection_map_path.str());

			cubics = global_map.segment;
			G = vector<vector<int>>(cubics.size());
			for(int i = 0;i<cubics.size();i++){
				G[i] = cubics[i].get_edge();
			}

            vehicleinfo_sub = nh.subscribe<hellocm_msgs::VehicleInfo>("/filtered_vehicleinfo",1,&LightControlNode::callback_vehicle_info, this);
            targets_sub = nh.subscribe<hmg_utils::Targets>("/targets",1,&LightControlNode::callback_targets, this);

            // state = 0;
            // flag = 0;
            // real_turnon_flag = 0;
            // nearest_segment = -1;
            // last_light_time = clock();

            flag_location = 0;
            flag_targets = 0;
            flag_dl;
            flag_turnon = 0;

            time_check(LIGHT_CONTROL, SET_DEBUG, "", time_debug);
            ROS_WARN("[light_control] loaded");
        }

        //lights_indicator 
        // 1 ==> left indicator on
        // 0 ==> indicator off
        // -1 ==> right indicator on

        //lights_hazard
        // 1 ==> on
        // 0 ==> off

        void callback_targets(const hmg_utils::Targets::ConstPtr & msg){
            time_check(LIGHT_CONTROL, START);

            current_task = msg->task;
            next_task = msg->next_task;
            motion = msg->motion;
            changing_dir = msg->changing_dir;
            targets_intersection = msg->intersection;

            // if(nearest_segment == -1) nearest_segment = current_segment;

            updateLightIndicatorByLocation();
            time_check(LIGHT_CONTROL, CHECK, "by location");
            updateLightIndicatorByTargets();
            time_check(LIGHT_CONTROL, CHECK, "by targets");
            updateLightIncidatorByDL();
            time_check(LIGHT_CONTROL, CHECK, "by dl");

            int flag_tmp;
            if(flag_location != 0){
                flag_tmp = flag_location;
            }
            else if(flag_targets != 0){
                flag_tmp = flag_targets;
            }
            else if(flag_dl != 0){
                flag_tmp = flag_dl;
            }
            else flag_tmp = 0;

            if(flag_tmp != 0){
                flag_turnon = flag_tmp;
                last_light_time = msg->header.stamp.toSec();
            }
            else{
                if(flag_turnon != 0){
                    if((msg->header.stamp.toSec() - last_light_time)  > keep_light_time){
                        flag_turnon = 0;
                        last_light_time = msg->header.stamp.toSec();
                    }
                }
            }

            ros::param::set("/light_indicator", flag_turnon);
            if(debug) ROS_INFO("[light_control] flag_location: %d, flag_targets: %d, flag_tmp: %d, flag_turnon: %d, duration : %lf",
                flag_location, flag_targets, flag_tmp, flag_turnon, (msg->header.stamp.toSec() - last_light_time)
            );
        }

        void callback_vehicle_info(const hellocm_msgs::VehicleInfo::ConstPtr &msg){
            // ROS_INFO("[light_control] vehicle info callback");
            x = msg->x[0];
            y = msg->x[1];
            vx = msg->v[0];
            vy = msg->v[1];
        }

        void updateLightIndicatorByLocation(){
            flag_location = 0;
            bool isIntersection = global_map.isIntersection(x,y);
            if(isIntersection){
                if((current_task == INTERSECTION_LEFT && targets_intersection) || next_task == INTERSECTION_LEFT){
                    flag_location = 1;
                }
                else if((current_task == INTERSECTION_RIGHT && targets_intersection) || next_task == INTERSECTION_RIGHT){
                    flag_location = -1;
                }
            }
        }

        void updateLightIndicatorByTargets(){
            flag_targets = -changing_dir;
        }

        void updateLightIncidatorByDL(){
            static int prev_flag_dl = 0;

            flag_dl = 0;
            bool isIntersection = global_map.isIntersection(x,y);
            if(isIntersection && (current_task == DRIVING_SECTION || targets_intersection)) return;

            vector<int> possible_segments = global_map.getSegmentArray(x, y);
            PoseState ps;
            ps.x = x; ps.y = y;
            ps.vx = vx; ps.vy = vy;
            int count[3] = {0,0,0};
            bool in_lane = false;
            for(int seg : possible_segments){
                SLState sl = cubics[seg].transform(ps);
                if(abs(sl.l) > check_l_range) continue;
                if(abs(sl.l) < l_range) in_lane= true;
                if(sl.dl < -dl_range) count[0]++;
                else if(sl.dl < dl_range) count[1]++;
                else count[2]++;
            }

            if(count[0] < count[2]) flag_dl = 1;
            else if(count[0] > count[2]) flag_dl = -1;
            else if(!in_lane) flag_dl = prev_flag_dl;
            else flag_dl = 0;

            if(flag_dl != 0) prev_flag_dl = flag_dl;
        } 
};


int main(int argc, char **argv){
    ros::init(argc, argv, "light_control_node");
    ros::param::get("/lcn_debug", debug);
    ros::param::get("/lcn_time_debug", time_debug);
    ros::param::get("/lcn_l_range", l_range);
    ros::param::get("/lcn_check_l_range", check_l_range);
    ros::param::get("/lcn_dl_range", dl_range);
    ros::param::get("/lcn_keep_light_time", keep_light_time);
    LightControlNode light_control_node;
    ros::spin();

    return 0;

}




