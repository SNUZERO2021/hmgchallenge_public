#include <iostream>
#include <sstream>
#include <vector>
#include <algorithm>
#include "tools.h"
#include "ros/ros.h"
#include "ros/package.h"
#include <geometry_msgs/Point.h>
#include "utils.h"
#include "GlobalMap.h"
#include "cubic_spline_2d.h"
#include "hmg_utils/PredictionArray.h"
#include "hmg_utils/PedestrianArray.h"
#include "hmg_utils/ConvexHullArray.h"
#include "hmg_utils/BoxArray.h"
#include "hellocm_msgs/VehicleInfo.h"
#include "hellocm_msgs/Speed_Limit.h"
#include "hellocm_msgs/TrafficLight.h"

using namespace std;

const double COST0 = 1e10;
const double COST1 = 1e8;
const double COST2 = 1e6;
const double COST3 = 1e4;
const double COST4 = 1e2;
const double COST5 = 1;


// decision maker
bool debug;
bool time_debug;
bool waypoint_mode;
bool waypoint_debug;

double normal_driving_speed_ratio;
double lane_change_speed_ratio;
double caution_speed_ratio;
double threshold_finish_inter;
double threshold_next_segment;
double threshold_stop_s;
double threshold_stop_ds;
double threshold_start_time;
double threshold_object_l_range;
double threshold_vehicle_l_range;
double threshold_pedestrian_l_range;
double threshold_complete_lc_l;
double threshold_complete_lc_dl;
double threshold_complete_lc_s;
int threshold_lane_change_count;
double pedestrian_time;
double pedestrian_safe;
double stop_safe;
double stop_v_s_ratio;
double stop_brake;
double braking_distance_offset;
double stop_fc_dist;

double left_l_offset;
double right_l_offset;

double o_safe;
double t_lc;
double lc_penalty;
double d_inter;
double d_minlc;
double d_avglc;
double jam_speed;

class RosNode{
	public:
		ros::NodeHandle n;
		ros::Subscriber vehicle_info_sub;
		ros::Subscriber prediction_sub;
		ros::Subscriber pedestrian_sub;
		ros::Subscriber obstacle_sub;
		ros::Subscriber speed_limit_sub;
		ros::Subscriber traffic_light_sub;
		ros::Publisher targets_pub;

		// subscribing data
		hmg_utils::PredictionArray pa;
		hmg_utils::PedestrianArray psa;
		hmg_utils::ConvexHullArray cha;
		double prev_speed_limit;
		double cur_speed_limit;
		double normal_driving_speed;
		double lane_change_speed;
		double caution_speed;

		// Global map
		GlobalMap global_map;			// to load Global map
		vector<CubicSpline2D> cubics;	// cubic spline segments
		vector<vector<int>> G;			// edge information
		vector<string> names;			// names		

		// Mission Recognizer
		MissionRecognizer mr;

		// Lane Change Handler
		LaneChangeHandler lch;

		// Light Handler
		LightHandler lh;

		// info
		int state;
		int task;
		int motion;
		int changing_dir;
		int current_segment;
		int current_lane;
		double current_s;
		vector<double> current_boundary;
		int light;
		double light_distance;

		int lane_change_count;
		int lane_change_prev_dir;

		bool fc_exist;
		Vehicle fc;
		vector<string> back_cars;
		bool pedestrian_exist;
		double stop_s_min;	// 보행자나 정지선 앞 

		// output
		hmg_utils::Targets ret;

	public:
		RosNode(){
			if(debug) ROS_WARN("[decision_maker] Debugging mode");

			stringstream segment_info_path;
			segment_info_path << ros::package::getPath("global_map") << "/config/segment_info.txt";
			stringstream segment_map_path;
    		segment_map_path << ros::package::getPath("global_map") << "/config/segment_map.txt";
			global_map = GlobalMap(10,0,13500,0,10,15000,24300,21400);
    		global_map.loadSegmentInfo(segment_info_path.str());
			global_map.loadSegmentMap(segment_map_path.str());
			cubics = global_map.segment;
			G = vector<vector<int>>(cubics.size());
			for(int i = 0;i<cubics.size();i++){
				G[i] = cubics[i].get_edge();
			}
			names = vector<string>(cubics.size());
			for(int i = 0;i<cubics.size();i++){
				names[i] = cubics[i].get_id()[1];
			}

			if(waypoint_mode){
				stringstream waypoint_path;
    			waypoint_path << ros::package::getPath("global_map") << "/waypoint/waypoint.csv";
				mr.loadWaypoint(global_map, cubics, names, waypoint_path.str(), waypoint_debug);
			}
			stringstream mission_info_path;
			mission_info_path << ros::package::getPath("global_map") << "/mission_config/mission_info.txt";
			mr.loadMissionInfo(mission_info_path.str(), names);
			mr.check_mission(names);
			current_segment = mr.init_segment;

			ROS_INFO("[decision_maker] init segment : %s %d", names[mr.init_segment].c_str(), mr.init_segment);

			state = NORMAL;
			motion = DRIVE;
			lane_change_count = 0;
			lane_change_prev_dir = 0;

			vehicle_info_sub = n.subscribe("/filtered_vehicleinfo", 1, &RosNode::VehicleInfoCallback, this);
			prediction_sub = n.subscribe("/prediction_array", 1, &RosNode::PredictionCallback, this);
			pedestrian_sub = n.subscribe("/pedestrian_array", 1, &RosNode::PedestrianCallback, this);
			obstacle_sub = n.subscribe("/static_obstacles", 1, &RosNode::ObstacleCallback, this);
			speed_limit_sub = n.subscribe("/speed_limit", 1, &RosNode::SpeedLimitCallback, this);
			traffic_light_sub = n.subscribe("/traffic_light", 1, &RosNode::TrafficLightCallback, this);

			targets_pub = n.advertise<hmg_utils::Targets>("/targets", 1);

			cur_speed_limit = 50.0;
			prev_speed_limit = 50.0;
			normal_driving_speed = cur_speed_limit * normal_driving_speed_ratio / 3.6;
			lane_change_speed = cur_speed_limit * lane_change_speed_ratio / 3.6;
			caution_speed = cur_speed_limit * caution_speed_ratio / 3.6;

			time_check(DECISION_MAKER, SET_DEBUG, "", time_debug);
		}

		void PredictionCallback(const hmg_utils::PredictionArray & msg){
			pa = msg;
		}

		void PedestrianCallback(const hmg_utils::PedestrianArray & msg){
			psa = msg;
		}

		void ObstacleCallback(const hmg_utils::ConvexHullArray & msg){
			cha = msg;
		}

		void SpeedLimitCallback(const hellocm_msgs::Speed_Limit & msg){
			if(msg.speed_limit < 5.0){
				normal_driving_speed = cur_speed_limit * normal_driving_speed_ratio / 3.6;
				lane_change_speed = cur_speed_limit * lane_change_speed_ratio / 3.6;
				caution_speed = cur_speed_limit * caution_speed_ratio / 3.6;
			}
			else{
				if(abs(msg.speed_limit - cur_speed_limit) > 5.0){
					prev_speed_limit = cur_speed_limit;
					cur_speed_limit = msg.speed_limit;
					double tmp_speed_limit = min(prev_speed_limit, cur_speed_limit);
					normal_driving_speed = tmp_speed_limit * normal_driving_speed_ratio / 3.6;
					lane_change_speed = tmp_speed_limit * lane_change_speed_ratio / 3.6;
					caution_speed = tmp_speed_limit * caution_speed_ratio / 3.6;
				}
			}			
		}

		void TrafficLightCallback(const hellocm_msgs::TrafficLight & msg){
			if(msg.id == -1) {
				light = -1;
			}
			else {
				light = msg.state;
				light_distance = msg.distance;
			}
		}

		void VehicleInfoCallback(const hellocm_msgs::VehicleInfo & msg){
			ROS_WARN("[decision_maker] VehicleInfo callback called");
			time_check(DECISION_MAKER, START);
			if(debug) ROS_WARN("[decision_maker] x : %lf, y : %lf", msg.x[0], msg.x[1]);
			/* 0. get current position */
			double current_x = msg.x[0];
			double current_y = msg.x[1];
			double current_yaw = msg.yaw;

			PoseState ps;
			SLState sl;
            ps.x = current_x;
            ps.y = current_y;
			ps.vx = msg.v[0];
			ps.vy = msg.v[1];

			/* 1. get current segment, sl and task using Mission Recognizer */
			if(motion == LANE_CHANGE){
				vector<int> near_segments = mr.near_segments(G[current_segment], G);
				int target_segment;
				if(changing_dir < 0)	{
					target_segment = near_segments[1];
				}
				else {
					target_segment = near_segments[2];
				}
				if(target_segment == -1){
					ROS_ERROR("[decision_maker] lane change state but no side lane");
					assert(false);
				}
				if(debug) ROS_WARN("[decision_maker] i am changing lane dir %d, target : %s", changing_dir, names[target_segment].c_str());
				sl = cubics[target_segment].transform(ps);
				if(debug) ROS_WARN("[decision_maker] was i changed lane? l : %lf, dl : %lf", sl.l, sl.dl);
				if(abs(sl.l) < threshold_complete_lc_l && abs(sl.dl) < threshold_complete_lc_dl) {
					if(debug) ROS_WARN("[decision_maker] yes i changed lane");
					current_segment = target_segment;
					setMotion(DRIVE);
				}
			}
			sl = cubics[current_segment].transform(ps);
			if(debug) ROS_WARN("[decision_maker] current segment : %s", names[current_segment].c_str());
			if(debug) ROS_WARN("[decision_maker] sl transformed s: %lf l: %lf", sl.s, sl.l);

			if(sl.s > cubics[current_segment].get_boundary()[2] - threshold_next_segment){
				if(debug) ROS_WARN("[decision_maker] I think I'm in next segment");
				vector<int> near_segments = mr.near_segments(G[current_segment], G);
				current_segment = near_segments[0];
				if(current_segment == -1){
					ROS_WARN("[decision_maker] Can't update to next segment ; no next segment");
					assert(false);
				}
				if(debug) ROS_WARN("[decision_maker] Update to next segment : %s", names[current_segment].c_str());
			}
			
			sl = cubics[current_segment].transform(ps);
			current_s = sl.s;
			double current_speed = sqrt(msg.v[0] * msg.v[0] + msg.v[1] * msg.v[1]);
			double current_l = sl.l;
			if(debug) ROS_WARN("[decision_maker] Finally, segment : %d %s, s: %lf l: %lf", current_segment, names[current_segment].c_str(), sl.s, sl.l);

			mr.update(current_segment);


			/* 2. processing info */

			// get current segment info
			int current_lane = G[current_segment][LANE_NUM1];
			vector<double> current_boundary = cubics[current_segment].get_boundary();


			// get next segment && update LaneChangeHandler
			int next_segment = mr.near_segments(G[current_segment], G)[0];
			if(next_segment == -1){
				int iter = 10;
				while(iter--) ROS_WARN("[decision_maker] no next segment");
				// assert(false);
			}
			
			if(next_segment != -1) lch.update(mr.ind, current_s, cubics[current_segment], cubics[next_segment]);
			else lch.update(mr.ind, current_s, cubics[current_segment]);


			// get side segments
			vector<int> left_segments;
			int tmp_left = mr.near_segments(G[current_segment], G)[1];
			while(tmp_left != -1){
				left_segments.push_back(tmp_left);
				int tmp_tmp_left = mr.near_segments(G[tmp_left], G)[1];
				if(find(left_segments.begin(), left_segments.end(), tmp_tmp_left) != left_segments.end()) break;
				tmp_left = tmp_tmp_left;
			}

			vector<int> right_segments;
			int tmp_right = mr.near_segments(G[current_segment], G)[2];
			while(tmp_right != -1){
				right_segments.push_back(tmp_right);
				int tmp_tmp_right = mr.near_segments(G[tmp_right], G)[2];
				if(find(right_segments.begin(), right_segments.end(), tmp_tmp_right) != right_segments.end()) break;
				tmp_right = tmp_tmp_right;
			}

			vector<Lane> lanes;
			Lane tmp_lane;
			for(int i = left_segments.size()-1;i>=0;i--){
				tmp_lane.segment = left_segments[i];
				tmp_lane.lane = current_lane - i - 1;
				SLState sl = cubics[tmp_lane.segment].transform(ps);
				tmp_lane.s = sl.s;
				tmp_lane.d_left = cubics[tmp_lane.segment].get_dist() - tmp_lane.s;
				tmp_lane.l_dist = mr.dist_to_desired_lanes(tmp_lane.lane);
				tmp_lane.dir = mr.dir_to_desired_lanes(tmp_lane.lane);
				lanes.push_back(tmp_lane);
			}
			tmp_lane.segment = current_segment;
			tmp_lane.lane = current_lane;
			tmp_lane.s = current_s;
			tmp_lane.d_left = cubics[tmp_lane.segment].get_dist() - tmp_lane.s;
			tmp_lane.l_dist = mr.dist_to_desired_lanes(tmp_lane.lane);
			tmp_lane.dir = mr.dir_to_desired_lanes(tmp_lane.lane);
			lanes.push_back(tmp_lane);
			for(int i = 0;i<right_segments.size();i++){
				tmp_lane.segment = right_segments[i];
				tmp_lane.lane = current_lane + i + 1;
				SLState sl = cubics[tmp_lane.segment].transform(ps);
				tmp_lane.s = sl.s;
				tmp_lane.d_left = cubics[tmp_lane.segment].get_dist() - tmp_lane.s;
				tmp_lane.l_dist = mr.dist_to_desired_lanes(tmp_lane.lane);
				tmp_lane.dir = mr.dir_to_desired_lanes(tmp_lane.lane);
				lanes.push_back(tmp_lane);
			}
			int lane_offset = left_segments.size();
			lanes.front().very_left = true;
			lanes.back().very_right = true;
			
			time_check(DECISION_MAKER, CHECK, "processing segment");

			// TODO : light, signal
			double braking_distance = (current_speed * current_speed / 2 / stop_brake);
			bool can_stop = braking_distance < (lanes[lane_offset].d_left - stop_safe);			
			lh.update(msg.header.stamp.toSec(), light, mr.current_task(), mr.next_task(), can_stop);
			bool go_signal = lh.go_signal;
			double duration_light_changed =  lh.duration_light_changed;

			// handle traffics
			fc_exist = false;
			
			// dynamic		
			for(hmg_utils::Prediction pred : pa.predictions){
				// if(debug) ROS_WARN("[decision_maker] dynamic object type %d segment %s ", pred.object_info.type, names[pred.segment_index].c_str());
				
				if(pred.object_info.type == 0){
					for(Lane & lane : lanes){
						if(lane.segment == pred.segment_index){
							bool in_lane = false;
							for(double l : pred.l_min){
								if(abs(l) < threshold_vehicle_l_range) in_lane = true;
							}
							for(double l : pred.l_max){
								if(abs(l) < threshold_vehicle_l_range) in_lane = true;
							}
							if(!in_lane) continue;

							lane.vehicles.push_back(Vehicle(pred.object_info.id, pred.s_min[0], pred.v[0]));
							break;
						}
					}
				}
				else{
					ROS_ERROR("[decision_maker] box object info type : %d", pred.object_info.type);
					assert(false);
				}
			}
			//////////////////////////////////////////////////////
			back_cars.clear();
			vector<hmg_utils::BoxArray> ba = global_map.PredictionService(pa.predictions, vector<double>(1,msg.header.stamp.toSec()));			
			if(!ba.empty()){
				for(hmg_utils::Box box : ba[0].boxes){
					double sumx=0, sumy=0;
					for(double _x : box.x) sumx += _x;
					for(double _y : box.y) sumy += _y;
					double avgx = sumx / box.x.size();
					double avgy = sumy / box.y.size();

					if((avgx - current_x) * cos(current_yaw) + (avgy - current_y) * sin(current_yaw) > 0) continue;
					back_cars.push_back(box.object_info.id);
				}
			}
			///////////////////////////////////////////////////////////
			
			// lane cars speed
			for(Lane & lane : lanes){
				if(!lane.vehicles.empty()) sort(lane.vehicles.begin(), lane.vehicles.end());
				if(lane.vehicles.empty()){
					lane.speed = normal_driving_speed;
				}
				else{
					double sum = 0;
					for(Vehicle & v : lane.vehicles){
						sum += v.speed;
					}
					double avg = sum / lane.vehicles.size();
					lane.speed = max(min(normal_driving_speed, avg), 1.0);
				}
			}

			// front car is stopped? check just current segment
			int fc_ind = 0;
			bool fc_obstacle = false;
			bool fc_stopped = false;
			for(;fc_ind<lanes[lane_offset].vehicles.size();fc_ind++) if(lanes[lane_offset].vehicles[fc_ind].s > current_s) break;
			if(fc_ind != lanes[lane_offset].vehicles.size()){
				if(debug) ROS_WARN("fc exist");
				fc_exist = true;
				fc = lanes[lane_offset].vehicles[fc_ind];

				double fc_speed = fc.speed;
				fc_stopped = abs(fc.speed) < threshold_stop_ds;
				if(fc_stopped){
					if(go_signal && duration_light_changed> threshold_start_time){
						fc_obstacle = true;
						fc_exist = false;
					}
					else{
						fc_obstacle = true;
						fc_exist = false;

						// 정지선 앞에 서있으면 일단 정상 정차한걸로 판단
						if(mr.is_current_last_sector(G[current_segment][CURRENT_SECTOR_NUM])){
							if(current_boundary[2] - fc.s < threshold_stop_s) {
								fc_obstacle = false;
								fc_exist = true;
							}
						}  
						else if(mr.is_current_last_sector(G[current_segment][CURRENT_SECTOR_NUM]+1)){
							if(current_boundary[3] - fc.s < threshold_stop_s) {
								fc_obstacle = false;
								fc_exist = true;
							}
						}

						// 혹은 앞앞차가 정지한 경우 정상 정차한걸로 판단
						if(lanes[lane_offset].vehicles.size() > fc_ind + 1){
							if(abs(lanes[lane_offset].vehicles[fc_ind+1].speed) < threshold_stop_ds ){
								fc_obstacle = false;
								fc_exist = true;
							}
						}
					}
				}
				if(debug) ROS_WARN("[decision_maker] front car speed %lf, front car is obstacle ? %s", fc_speed, (bool_to_string(fc_obstacle)));
			}
			else{
				fc_exist = false;
			}
			
			// 앞차를 장애물로 고려
			if(fc_obstacle){
				lanes[lane_offset].obstacle_exist = true;
				lanes[lane_offset].obstacle_dist = fc.s - current_s;
			}

			// 다른 차선의 멈춘 차들
			for(Lane & lane : lanes){
				for(Vehicle & v : lane.vehicles){
					if(v.s < lane.s) continue;
					if(abs(v.speed) < threshold_stop_ds){
						lane.obstacle_exist = true;
						lane.obstacle_dist = min(lane.obstacle_dist, v.s - lane.s);
					}
				}
			}
			

			time_check(DECISION_MAKER, CHECK, "processing dynamic");

			// 보행자, stop_s_min 설정
			pedestrian_exist = false;
			stop_s_min = INF;			

			for(hmg_utils::Pedestrian & pedestrian : psa.predictions){
				// if(debug) ROS_WARN("[decision_maker] pedestrian type %d id %s ", pedestrian.object_info.type, pedestrian.object_info.id.c_str());
				if(pedestrian.t.empty()) continue;
				PoseState ps;
				ps.x = pedestrian.x.front();
				ps.y = pedestrian.y.front();
				ps.vx = pedestrian.vx.front();
				ps.vy = pedestrian.vy.front();
				
				SLState sl_cur = cubics[current_segment].transform(ps, false);
				if(sl_cur.s > current_s - 4.0){
					double pedestrian_l_min = min(sl_cur.l, sl_cur.l + sl_cur.dl * pedestrian_time);
					double pedestrian_l_max = max(sl_cur.l, sl_cur.l + sl_cur.dl * pedestrian_time);

					if(pedestrian_l_min < threshold_pedestrian_l_range && pedestrian_l_max > -threshold_pedestrian_l_range){
						pedestrian_exist = true;
						stop_s_min = min(stop_s_min, sl_cur.s);
						stop_s_min = min(stop_s_min, sl_cur.s + sl_cur.ds * pedestrian_time);
					}
				}
				
				if(motion == LANE_CHANGE && lane_offset+changing_dir >= 0 && lane_offset+changing_dir < lanes.size()){
					SLState sl_side = cubics[lanes[lane_offset+changing_dir].segment].transform(ps, false);
					if(sl_side.s > lanes[lane_offset+changing_dir].s - 4.0){
						double pedestrian_l_min = min(sl_side.l, sl_side.l + sl_side.dl * pedestrian_time);
						double pedestrian_l_max = max(sl_side.l, sl_side.l + sl_side.dl * pedestrian_time);

						if(pedestrian_l_min < threshold_pedestrian_l_range && pedestrian_l_max > -threshold_pedestrian_l_range){
							pedestrian_exist = true;
							stop_s_min = min(stop_s_min, sl_side.s);
							stop_s_min = min(stop_s_min, sl_side.s + sl_side.ds * pedestrian_time);
						}
					}
				}
			}
			stop_s_min -= pedestrian_safe;

			if(mr.current_task() == DRIVING_SECTION){
				if(mr.is_current_last_sector(G[current_segment][CURRENT_SECTOR_NUM])){
					stop_s_min = min(stop_s_min, current_boundary[2] - stop_safe);
					// printf("stop_s_min : %lf, current_boundary[2] : %lf, stop_safe : %lf\n\n", stop_s_min, current_boundary[2], stop_safe);
				}  
				else if(mr.is_current_last_sector(G[current_segment][CURRENT_SECTOR_NUM]+1)){
					stop_s_min = min(stop_s_min, current_boundary[3] - stop_safe);
					// printf("stop_s_min : %lf, current_boundary[3] : %lf, stop_safe : %lf\n\n", stop_s_min, current_boundary[3], stop_safe);
				}
			}
			// printf("stop_s_min : %lf\n\n", stop_s_min);
			
			// 앞차가 신호로 정지해있으면 이것도 stop_s_min 고려
			if(fc_exist && fc_stopped) stop_s_min = min(stop_s_min, fc.s - stop_fc_dist);

			time_check(DECISION_MAKER, CHECK, "processing pedestrian");

			// static
			for(hmg_utils::ConvexHull & cv : cha.obstacles){
				for(Lane &lane : lanes){
					double s_min = INF;
					double s_max = -INF;
					vector<SLState> cv_sl;
					for(int i = 0;i<cv.count;i++){
						PoseState ps;
						ps.x = cv.points[i].x;
						ps.y = cv.points[i].y;
						cv_sl.push_back(cubics[lane.segment].transform(ps, false));
					}
					for(int i = 0;i<cv.count;i++){
						SLState sl1 = cv_sl[i];
						SLState sl2 = cv_sl[(i+1)%cv.count];

						if(sl1.l < threshold_object_l_range && sl1.l > -threshold_object_l_range){
							s_min = min(s_min, sl1.s);
							s_max = max(s_max, sl1.s);
						}
						if((sl1.l - threshold_object_l_range) * (sl2.l - threshold_object_l_range) < 0){
							double s_avg = sl1.s + (threshold_object_l_range - sl1.l) * (sl2.s - sl1.s) / (sl2.l - sl1.l);
							s_min = min(s_min, s_avg);
							s_max = max(s_max, s_avg);
						}
						if((sl1.l + threshold_object_l_range) * (sl2.l + threshold_object_l_range) < 0){
							double s_avg = sl1.s + (-threshold_object_l_range - sl1.l) * (sl2.s - sl1.s) / (sl2.l - sl1.l);
							s_min = min(s_min, s_avg);
							s_max = max(s_max, s_avg);
						}
					}
					if(s_min > s_max) continue;
					if(s_min < lane.s && lane.s < s_max){
						lane.obstacle_exist = true;
						lane.obstacle_dist = 0;
					}
					else if(lane.s < s_min){
						lane.obstacle_exist = true;
						lane.obstacle_dist = s_min - lane.s;
					}
				}
			}

			time_check(DECISION_MAKER, CHECK, "processing static");

			// calculate d_left, ttrd
			double d_sum_left = 0;
			double t_sum_left = 0;
			double min_obstacle_dist_left = INF;
			bool jam_left = false;
			for(int i = lane_offset-1;i>=0;i--){
				d_sum_left += max(3.0, lanes[i].speed * t_lc);
				t_sum_left += t_lc + lc_penalty;
				if(motion == LANE_CHANGE && changing_dir == -1 && i == lane_offset-1) {
					lanes[i].ttrd = 0;
				}
				else{
					lanes[i].d_left -= d_sum_left;
					lanes[i].ttrd = t_sum_left;
				}
				
				// not busy
				if(lanes[i].d_left > d_inter + d_avglc * lanes[i].l_dist){
					lanes[i].ttrd += (lanes[i].d_left - d_inter - d_avglc*lanes[i].l_dist) / lanes[i].speed; 
				}

				double tmp_d_left = min(lanes[i].d_left , d_inter + d_avglc * lanes[i].l_dist);
				int j = 0;
				int dir = lanes[i].dir;
				while(j < lanes[i].l_dist){
					j++;
					if(i+j*dir < 0 || i+j*dir >= lanes.size()){
						j--;
						break;
					}
					tmp_d_left -= max(3.0, lanes[i+j*dir].speed * t_lc);
					lanes[i].ttrd += t_lc + lc_penalty;
				}
				if(tmp_d_left > 0) lanes[i].ttrd += tmp_d_left / lanes[i+j*dir].speed;
				
				lanes[i].obstacle_dist -= d_sum_left;

				min_obstacle_dist_left = min(min_obstacle_dist_left, lanes[i].obstacle_dist);
				lanes[i].obstacle_dist = min(min_obstacle_dist_left, lanes[i].obstacle_dist);

				lanes[i].jam = jam_left;
				if(lanes[i].speed < jam_speed) jam_left = true;
			}

			double d_sum_right = 0;
			double t_sum_right = 0;
			double min_obstacle_dist_right = INF;
			bool jam_right = false;
			for(int i = lane_offset+1;i<lanes.size();i++){
				d_sum_right += max(3.0, lanes[i].speed * t_lc);
				t_sum_right += t_lc + lc_penalty;
				if(motion == LANE_CHANGE && changing_dir == 1 && i == lane_offset+1) {
					lanes[i].ttrd = 0;
				}
				else{
					lanes[i].d_left -= d_sum_right;
					lanes[i].ttrd = t_sum_right;
				}

				// not busy
				if(lanes[i].d_left > d_inter + d_avglc * lanes[i].l_dist){
					lanes[i].ttrd += (lanes[i].d_left - d_inter - d_avglc*lanes[i].l_dist) / lanes[i].speed; 
				}

				double tmp_d_left = min(lanes[i].d_left , d_inter + d_avglc * lanes[i].l_dist);
				int j = 0;
				int dir = lanes[i].dir;
				while(j < lanes[i].l_dist){
					j++;
					if(i+j*dir < 0 || i+j*dir >= lanes.size()){
						j--;
						break;
					}
					tmp_d_left -= max(3.0, lanes[i+j*dir].speed * t_lc);
					lanes[i].ttrd += t_lc + lc_penalty;
				}
				if(tmp_d_left > 0) lanes[i].ttrd += tmp_d_left / lanes[i+j*dir].speed;
				
				lanes[i].obstacle_dist -= d_sum_right;

				min_obstacle_dist_right = min(min_obstacle_dist_right, lanes[i].obstacle_dist);
				lanes[i].obstacle_dist = min(min_obstacle_dist_right, lanes[i].obstacle_dist);

				lanes[i].jam = jam_right;
				if(lanes[i].speed < jam_speed) jam_right = true;
			}
			lanes[lane_offset].ttrd = lanes[lane_offset].d_left / lanes[lane_offset].speed;

			time_check(DECISION_MAKER, CHECK, "processing lanes");
			

			/* determine state, task, motion, targets */
			ret = hmg_utils::Targets();
			ret.header = msg.header;
			setTask(mr.current_task(), mr.next_task());
			ret.s_min = stop_s_min;
			ret.current_segment = current_segment;
			ret.next_segment = next_segment;
			ret.changing_segment = -1;

			if(mr.current_task() != DRIVING_SECTION && current_s < current_boundary[2] - threshold_finish_inter){
				if(debug) ROS_WARN("[decision_maker] intersection situation");
				ret.intersection = true;
				// normal
				// drive / caution
				// target cur

				if(pedestrian_exist){
					setMotion(CAUTION);
					ret.s_min = stop_s_min;
					ret.changing_dir = 0;
					setTargetStop(lanes[lane_offset]);
					finish();
					return;
				}
				else{
					setState(NORMAL);
					ret.changing_dir = 0;
					setTargetCur(lanes[lane_offset]);
					finish();
					return;
				}
			}
			ret.intersection = false;

			if(lanes[lane_offset].d_left < d_inter + d_minlc * lanes[lane_offset].l_dist){
				if(debug) ROS_WARN("[decision_maker] urgent situation");
				// urgent
				// drive / caution / stop
				// target cur / target lane change

				setState(URGENT);
			}

			else if(lanes[lane_offset].d_left < d_inter + d_avglc * lanes[lane_offset].l_dist){
				// busy
				if(debug) ROS_WARN("[decision_maker] busy situation");
				setState(BUSY);

				// 선호 차선 방면 말고 제외
				for(Lane & lane : lanes){
					if(lane.l_dist > lanes[lane_offset].l_dist){
						lane.cost += COST0;
					}
				}
			}
			else{
				// normal
				if(debug) ROS_WARN("[decision_maker] normal situation");
				setState(NORMAL);
			}

			for(Lane & lane : lanes){
				// urgent 차로
				if(lane.d_left < d_inter + d_minlc * lane.l_dist){
					lane.cost += COST1;
				}

				// 가까운 장애물
				if(lane.obstacle_exist && lane.obstacle_dist < o_safe){
					lane.cost += COST2;
				}
				else if(lane.jam){
					lane.cost += COST2;
				}

				// 장애물 존재
				if(lane.obstacle_exist){
					lane.cost += COST3;
				}

				// busy 차로
				if(lane.d_left < d_inter + d_avglc * lane.l_dist){
					lane.cost += COST4;
				}				

				// 속도
				lane.cost += 100 - 50.0/max(1.0,lane.ttrd);

				// 선호 차선, 현채 차선, 반대 차선 cost 차등 분배 (속도에 비해서)
				if(lane.l_dist > lanes[lane_offset].l_dist){
					lane.cost += COST5;
				}
				else if(lane.l_dist < lanes[lane_offset].l_dist){
					lane.cost -= COST5;
				}
			}
			
			double min_cost = INF;
			int min_index = -1;
			for(int i = 0;i<lanes.size();i++){
				if(lanes[i].cost < min_cost){
					min_cost = lanes[i].cost;
					min_index = i;
				}
			}

			if(min_index == -1){
				ROS_ERROR("[decision_maker] what happen? min index = -1");
				print_lanes(names, lanes);
				assert(false);
			}

			if(debug){
				print_lanes(names, lanes);
				ROS_WARN("[decision_maker] min cost segment : %s", names[lanes[min_index].segment].c_str());
			}

			if(!go_signal && (stop_s_min - stop_safe - current_s) < braking_distance + braking_distance_offset){
				if(debug) ROS_WARN("[decision_maker] not go signal situation");

				if(motion == LANE_CHANGE && lane_offset+changing_dir >= 0 && lane_offset+changing_dir < lanes.size()){
					int target_segment = lanes[lane_offset+changing_dir].segment;
					if(state == URGENT) {
						current_segment = target_segment;
					}
					// 차선 변경 중 보행자 나타날 시 더 가까운 차선을 내 차선으로 인지
					else{
						PoseState ps;
						ps.x = current_x;
						ps.y = current_y;
						ps.vx = msg.v[0];
						ps.vy = msg.v[1];

						SLState sl_cur = cubics[current_segment].transform(ps);
						SLState sl_target = cubics[target_segment].transform(ps);
						if(sl_target.l < sl_cur.l) {
							current_segment = target_segment;
							current_s = sl_target.s;
						}
					}
				}

				setMotion(STOP);
				// 앞차 있으면 앞차 추종
				// 거리에 따라 다른 숙도 추종
				ret.s_min = stop_s_min;
				ret.changing_dir = 0;
				setTargetStop(lanes[lane_offset]);
				finish();
				return;
			}
			else if(pedestrian_exist){
				if(debug) ROS_WARN("[decision_maker] pedestrian exist situation");
				
				if(motion == LANE_CHANGE && lane_offset+changing_dir >= 0 && lane_offset+changing_dir < lanes.size()){
					int target_segment = lanes[lane_offset+changing_dir].segment;
					if(state == URGENT) {
						current_segment = target_segment;
					}
					// 차선 변경 중 보행자 나타날 시 더 가까운 차선을 내 차선으로 인지
					else{
						PoseState ps;
						ps.x = current_x;
						ps.y = current_y;
						ps.vx = msg.v[0];
						ps.vy = msg.v[1];

						SLState sl_cur = cubics[current_segment].transform(ps);
						SLState sl_target = cubics[target_segment].transform(ps);
						if(sl_target.l < sl_cur.l) {
							current_segment = target_segment;
							current_s = sl_target.s;
						}
					}
				}

				setMotion(CAUTION);
				ret.changing_dir = 0;
				ret.s_min = stop_s_min;
				setTargetStop(lanes[lane_offset]);
				finish();
				return;
			}
			else if(state == URGENT){
				setMotion(DRIVE);
				setTargetCur(lanes[lane_offset]);
				if(!mr.in_desired_lanes(current_lane)){
					changing_dir = mr.dir_to_desired_lanes(current_lane);
					if(changing_dir == 1 && lanes.size() > lane_offset + 1){
						setMotion(LANE_CHANGE);
						setTargetLaneChange(lanes[lane_offset+1]);
						ret.changing_dir = changing_dir;
					}
					else if(changing_dir == -1 && lane_offset > 0){
						setMotion(LANE_CHANGE);
						setTargetLaneChange(lanes[lane_offset-1]);
						ret.changing_dir = changing_dir;
					}
					else{
						ROS_ERROR("[decision_maker] not in desired lanes but changing dir = 0 ???");
						print_lanes(names, lanes);
						assert(false);
					}
				}
				finish();
				return;
			}
			else if(state == BUSY || lanes[lane_offset].obstacle_exist || (motion == DRIVE && lch.get_dist() > threshold_complete_lc_s)){
				// 현재 차선이 min cost
				if(min_index == lane_offset){
					if(debug) ROS_WARN("[decision_maker] keep lane situation");
					ret.changing_dir = 0;
					setMotion(DRIVE);
					setTargetCur(lanes[lane_offset]);
					finish();
					return;
				}
				changing_dir = (min_index < lane_offset) ? (-1) : (1);
				ret.changing_dir = changing_dir;
				if(changing_dir == lane_change_prev_dir) lane_change_count++;
				else {
					lane_change_prev_dir = changing_dir;
					lane_change_count = 0;
				}
				// min cost 차선 방향으로 차선 변경
				if(lane_change_count > threshold_lane_change_count){
					if(debug) ROS_WARN("[decision_maker] lane change start situation");
					setMotion(LANE_CHANGE);
					setTargetCur(lanes[lane_offset]);
					if(lane_offset+changing_dir < 0 || lane_offset+changing_dir >= lanes.size()){
						ROS_ERROR("[decision_maker] lane_offset %d min index %d changing dir %d lane size %d ???", lane_offset, min_index, changing_dir, (int)lanes.size());
						print_lanes(names, lanes);
						assert(false);
					}
					setTargetLaneChange(lanes[lane_offset + changing_dir]);
					finish();
					return;
				}
				// 차선 변경 보류중
				else{
					setMotion(DRIVE);
					setTargetCur(lanes[lane_offset]);
					finish();
					return;
				}
			}
			else{
				// 차선 변경 중
				if(motion == LANE_CHANGE){
					if(debug) ROS_WARN("[decision_maker] lane change continue situation");
					setMotion(LANE_CHANGE);
					ret.changing_dir = changing_dir;
					setTargetCur(lanes[lane_offset]);
					if(lane_offset+changing_dir >= 0 && lane_offset+changing_dir < lanes.size()){
						setTargetLaneChange(lanes[lane_offset + changing_dir]);
					}
					else{
						ROS_ERROR("[decision_maker] lane_offset %d min index %d changing dir %d lane size %d ???", lane_offset, min_index, changing_dir, (int)lanes.size());
						print_lanes(names, lanes);
						assert(false);
					}
					finish();
					return;
				}
				// 차선 변경 안정화 중
				else{
					if(debug) ROS_WARN("[decision_maker] lane change stabilize situation");
					ret.changing_dir = 0;
					setMotion(DRIVE);
					setTargetCur(lanes[lane_offset]);
					finish();
					return;
				}
			}
			
		}

		void finish(){
			///////////////////////////////////////////
			for(hmg_utils::Target t : ret.targets){
				string id1 = t.target_front;
				while(find(back_cars.begin(), back_cars.end(), id1) != back_cars.end()){
					back_cars.erase(find(back_cars.begin(), back_cars.end(), id1));
				}
				string id2 = t.target_back;
				while(find(back_cars.begin(), back_cars.end(), id2) != back_cars.end()){
					back_cars.erase(find(back_cars.begin(), back_cars.end(), id2));
				}
			}
			ret.back_cars = back_cars;
			///////////////////////////////////////////
			if(motion == DRIVE || motion == LANE_CHANGE) ret.s_min = INF;
			targets_pub.publish(ret);
			if(debug){
				string flagStr[3] = {"Velocity", "Target id", "Target point"};
				for(hmg_utils::Target t : ret.targets){
					ROS_WARN("[decision_maker] flag : %s, segment : %s, s : %lf, v : %lf, front_id : %s, back_id : %s, priority : %d\n", 
							flagStr[t.flag].c_str(), names[current_segment].c_str(),t.s, t.velocity, t.target_front.c_str(), t.target_back.c_str(), t.priority); 
				}
			}
			if(debug) ROS_WARN("[decision_maker] ---------------------------------------------");
			time_check(DECISION_MAKER, CHECK, "publish target");
		}

		void setState(int _state){
			ret.state = _state;
			state = _state;
		}

		void setTask(int _task, int _next_task){
			task = _task;
			ret.task = _task;
			ret.next_task = _next_task;
		}

		void setMotion(int _motion){
			ret.motion = _motion;
			motion = _motion;
		}

		void setTargetCur(Lane & lane){
			double driving_speed = normal_driving_speed;
			if(fc_exist) driving_speed = min(driving_speed, fc.speed);
			if(task == INTERSECTION_RIGHT) driving_speed = min(driving_speed, caution_speed);
			else if(task == INTERSECTION_LEFT) driving_speed = min(driving_speed, caution_speed);

			double l_offset = determineLoffset(lane);

			if(debug) ROS_WARN("[decision_maker] target_cur %s", names[current_segment].c_str());
			if(debug) ROS_WARN("[decision_maker] driving speed %lf", driving_speed);
			if(fc_exist){
				hmg_utils::Target target_default = _Target(0, current_segment, l_offset, -1, driving_speed, "", "", 5);
				ret.targets.push_back(target_default);
				hmg_utils::Target target_cur = _Target(1, current_segment, l_offset, -1, driving_speed, fc.id, "", 10);
				ret.targets.push_back(target_cur);
			}
			else{
				hmg_utils::Target target_default = _Target(0, current_segment, l_offset, -1, driving_speed, "", "", 5);
				ret.targets.push_back(target_default);			
			}
		}

		void setTargetLaneChange(Lane & lane){
			if(debug) ROS_WARN("[decision_maker] target_lane_change %s", names[lane.segment].c_str());
			if(debug) ROS_WARN("[decision_maker] lane change speed %lf", lane_change_speed);
			int target_segment = lane.segment;
			ret.changing_segment = target_segment;
			double l_offset = determineLoffset(lane);
			hmg_utils::Target target_default = _Target(0, target_segment, l_offset, -1, lane_change_speed, "", "", 15);
			ret.targets.push_back(target_default);
			if(!lane.vehicles.empty()){
				int current_pos = -1;
				for(;current_pos<lane.vehicles.size()-1;current_pos++) {
					if(lane.s < lane.vehicles[current_pos+1].s) {
						break;
					}
				}
				for(int pos = current_pos -1; pos<=current_pos+1; pos++){
					if(pos < -1) continue;
					if(pos == -1){
						double target_speed = min(normal_driving_speed, lane.vehicles[0].speed);
						if(debug) ROS_WARN("[decision_maker] lane change speed pos == -1 %lf", target_speed);
						hmg_utils::Target target = _Target(1, target_segment, l_offset, -1, target_speed, lane.vehicles[0].id, "", 20);
						ret.targets.push_back(target);
					}
					else if(pos == lane.vehicles.size()-1){
						if(debug) ROS_WARN("[decision_maker] lane change speed pos == last %lf", lane_change_speed);
						hmg_utils::Target target = _Target(1, target_segment, l_offset, -1, lane_change_speed, "", lane.vehicles.back().id, 20);
						ret.targets.push_back(target);
					}
					else{
						double target_speed = min(normal_driving_speed, lane.vehicles[pos+1].speed);
						if(debug) ROS_WARN("[decision_maker] lane change speed pos == mid %lf", target_speed);
						hmg_utils::Target target = _Target(1, target_segment, l_offset, -1, target_speed, lane.vehicles[pos+1].id, lane.vehicles[pos].id, 20);
						ret.targets.push_back(target);
					}
				}
			}
		}

		void setTargetStop(Lane & lane){
			if(debug) ROS_WARN("[decision_maker] target stop segment %s s_min %lf",names[current_segment].c_str(), stop_s_min);
			double l_offset = determineLoffset(lane);
			if(fc_exist){
				hmg_utils::Target target_cur = _Target(1, current_segment, l_offset, -1, 0, fc.id, "", 10);
				ret.targets.push_back(target_cur);
			}
			else{
				hmg_utils::Target target_cur = _Target(2, current_segment, l_offset, stop_s_min, 0, "", "", 25);
				ret.targets.push_back(target_cur);
			}
			hmg_utils::Target target_default = _Target(0, current_segment, l_offset, -1, min<double>(max<double>(stop_v_s_ratio * (stop_s_min - current_s - 1.0), 0), caution_speed), fc.id, "", 5);
			ret.targets.push_back(target_default);
		}

		double determineLoffset(Lane & lane){
			if(task == INTERSECTION_RIGHT){
				return right_l_offset;
			}
			else if(task == INTERSECTION_LEFT){
				return left_l_offset;
			}
			else if(task == INTERSECTION_STRAIGHT){
				return 0;
			}
			else{
				if(lane.very_left && lane.very_right) return 0;
				else if(lane.very_left) return left_l_offset;
				else if(lane.very_right) return right_l_offset;
				else return 0;
			}
		}
};

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "decision_maker");
	ROS_WARN("dm init");
	ros::param::get("/dm_debug", debug);
	ros::param::get("/dm_time_debug", time_debug);
	ros::param::get("/dm_waypoint_mode", waypoint_mode);
	ros::param::get("/dm_waypoint_debug", waypoint_debug);
	ros::param::get("/dm_normal_driving_speed_ratio", normal_driving_speed_ratio);
	ros::param::get("/dm_lane_change_speed_ratio", lane_change_speed_ratio);
	ros::param::get("/dm_caution_speed_ratio", caution_speed_ratio);
	ros::param::get("/dm_threshold_finish_inter", threshold_finish_inter);
	ros::param::get("/dm_threshold_next_segment", threshold_next_segment);
	ros::param::get("/dm_threshold_stop_s", threshold_stop_s);
	ros::param::get("/dm_threshold_stop_ds",threshold_stop_ds);
	ros::param::get("/dm_threshold_start_time", threshold_start_time);
	ros::param::get("/dm_threshold_object_l_range", threshold_object_l_range);
	ros::param::get("/dm_threshold_vehicle_l_range", threshold_vehicle_l_range);
	ros::param::get("/dm_threshold_pedestrian_l_range", threshold_pedestrian_l_range);
	ros::param::get("/dm_threshold_complete_lc_l", threshold_complete_lc_l);
	ros::param::get("/dm_threshold_complete_lc_dl", threshold_complete_lc_dl);
	ros::param::get("/dm_threshold_complete_lc_s", threshold_complete_lc_s);
	ros::param::get("/dm_threshold_lane_change_count", threshold_lane_change_count);
	ros::param::get("/dm_pedestrian_time", pedestrian_time);
	ros::param::get("/dm_pedestrian_safe", pedestrian_safe);
	ros::param::get("/dm_stop_safe", stop_safe);
	ros::param::get("/dm_stop_v_s_ratio", stop_v_s_ratio);
	ros::param::get("/dm_stop_brake", stop_brake);
	ros::param::get("/dm_braking_distance_offset", braking_distance_offset);
	ros::param::get("/dm_stop_fc_dist", stop_fc_dist);

	ros::param::get("/dm_left_l_offset", left_l_offset);
	ros::param::get("/dm_right_l_offset", right_l_offset);
	
	ros::param::get("/dm_o_safe", o_safe);
	ros::param::get("/dm_t_lc", t_lc);
	ros::param::get("/dm_lc_penalty", lc_penalty);
	ros::param::get("/dm_d_inter", d_inter);
	ros::param::get("/dm_d_minlc", d_minlc);
	ros::param::get("/dm_d_avglc", d_avglc);
	ros::param::get("/dm_jam_speed", jam_speed);

	RosNode rosnode;
	ros::spin();

/*
	ros::Rate loop(100);
	while(1)
	{
		ros::spinOnce();
		
		// do something
		
		loop.sleep();
	}
*/
	return 0;
}
