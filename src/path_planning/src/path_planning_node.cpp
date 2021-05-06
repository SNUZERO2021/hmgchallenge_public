#include "frenet_optimal_trajectory.h"
#include "frenet_path.h"
#include "cpp_struct.h"
#include "pure_pursuit.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "ros/package.h"
// #include "nav_msgs/Odometry.h"
// #include "sensor_msgs/Imu.h"
#include "hellocm_msgs/VehicleInfo.h"
#include "hmg_utils/NearestSegment.h"
#include "hmg_utils/Targets.h"
#include "hmg_utils/Target.h"
#include "hmg_utils/PredictionArray.h"
#include "hmg_utils/PedestrianArray.h"
#include "hmg_utils/ConvexHullArray.h"
#include "hmg_utils/ObjectId.h"
#include "opencv2/opencv.hpp"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "utils.h"
#include "GlobalMap.h"
#include <ctime>
#include <sstream>

#include <vector>
#include <tuple>
#include <cmath>
#include "math.h"

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

using namespace std;

double max_steer_diff;
double safe_0;
double safe_1;
bool debug;
bool visual;
bool time_debug;

class PathPlanningNode{
 public:
    ros::NodeHandle n_;
    // ros::Subscriber odometry_sub;
    // ros::Subscriber imu_sub;
    ros::Subscriber vehicleinfo_sub;
    ros::Subscriber targets_sub;
    ros::Subscriber prediction_sub;
    ros::Subscriber pedestrian_sub;
	ros::Subscriber obstacle_sub;
    ros::Subscriber object_id_sub;
    FrenetHyperparameters fot_hp;
    FrenetInitialConditions fot_ic;
    // FrenetOptimalTrajectory fot;
    PoseState ps;
    FrenetPath best_frenet_path;
    CubicSpline2D *csp;
    Car car;
    vector<vector<ConvexHull>> dynamic_cvh_arrays;
    vector<ConvexHull> static_cvh_array;

    bool stop = false;
    int last_fail_count = 0;
    int last_succeed_count = 0;
    
    // subscribing data
    hmg_utils::PredictionArray pa;
    hmg_utils::PedestrianArray psa;
    hmg_utils::ConvexHullArray cha;
    vector<string> prv_id;
    vector<string> cur_id;

    // Global map
    GlobalMap global_map;			// to load Global map
    vector<CubicSpline2D> cubics;	// cubic spline segments
    vector<vector<int>> G;			// edge information (0 : s, 1 : nl, 2 : nr, 3 : l, 4 : r, 5 : inter(1)/not(0), 6 : lane, 7 : secondlane(inter))

    vector<double> prev_pos_x, prev_pos_y;

    // decision maker
    int task=0;
    int state=0;
    int motion=0;


    PathPlanningNode(FrenetHyperparameters fot_hp_){
        fot_hp = fot_hp_;
        vehicleinfo_sub = n_.subscribe("/filtered_vehicleinfo",1,&PathPlanningNode::vehicleinfoCallback,this);
        // odometry_sub = n_.subscribe("/Odometry",1,&PathPlanningNode::odometryCallback,this);
        // imu_sub = n_.subscribe("/Imu",1,&PathPlanningNode::imuCallback,this);
        targets_sub = n_.subscribe("/targets",1,&PathPlanningNode::targetsCallback,this);
        prediction_sub = n_.subscribe("/prediction_array", 1, &PathPlanningNode::PredictionCallback, this);
        pedestrian_sub = n_.subscribe("/pedestrian_array", 1, &PathPlanningNode::PedestrianCallback, this);
        obstacle_sub = n_.subscribe("/static_obstacles", 1, &PathPlanningNode::ObstacleCallback, this);
        object_id_sub = n_.subscribe("/object_id", 1, &PathPlanningNode::ObjectIdCallback, this);

        ros::param::get("/vehicle_front_length",car.front_length);
        ros::param::get("/vehicle_rear_length",car.rear_length);
        ros::param::get("/steering_ratio", car.steering_ratio);
        ros::param::get("/understeer_gradient", car.understeer_gradient);

        fot_ic.s0 = 0;
        fot_ic.c_speed = 0;
        fot_ic.c_accel = 0;
        fot_ic.c_d = 0;
        fot_ic.c_d_d = 0;
        fot_ic.c_d_dd = 0;
        fot_ic.target_speed = 10.0;
        // fot_ic.no = 0;
        // fot_ic.nw = 0;

        ps.ax=0;
        ps.ay=0;

        stringstream segment_info_path;
        segment_info_path << ros::package::getPath("global_map") << "/config/segment_info.txt";
        global_map = GlobalMap(2430, 2140, 1, 0, 1350, 0,1, 1500);
        global_map.loadSegmentInfo(segment_info_path.str());
        cubics = global_map.segment;
        G = vector<vector<int>>(cubics.size());
        for(int i = 0;i<cubics.size();i++){
            G[i] = cubics[i].get_edge();
        }

        time_check(PATH_PLANNING, SET_DEBUG, "", time_debug);
    }

    inline bool is_same_box(const hmg_utils::Box& box1, const hmg_utils::Box& box2){
        if((box1.segment_index == box2.segment_index) && (box1.object_info.id == box2.object_info.id)) return true;
        return false;
    }

    inline vector<vector<ConvexHull>> getConvexHull(const vector<hmg_utils::BoxArray>& predictions, const vector<string>& back_cars){
        /*
        input
        predictions : vector of BoxArray.
        boxes : BoxArray, boxes for certain time.

        output
        rt : vector of ConvexHull vector
        rt[t] : array of ConvexHull. each convexhull represents object movement
        object movement means convexhull which merged with same object in time times[t] and times[t+1].
        times size is the same as size of boxes.
        same object means same objec id and same segment index.
        */
        int n = predictions.size();
        vector<vector<ConvexHull>> rt;


        for(int i=0;i<n-1;i++){
            vector<ConvexHull> convexhull_array;
            for(hmg_utils::Box box1 : predictions[i].boxes){
                if(find(back_cars.begin(),back_cars.end(),box1.object_info.id) != back_cars.end()) continue;
                for(hmg_utils::Box box2 : predictions[i+1].boxes){
                    if(is_same_box(box1, box2)){
                        vector<point> p;
                        int a = box1.x.size();
                        int b = box2.x.size();
                        for(int k=0;k<a;k++) p.push_back(point(box1.x[k],box1.y[k]));
                        for(int k=0;k<b;k++) p.push_back(point(box2.x[k],box2.y[k]));
                        convexhull_array.push_back(ConvexHull(p, box1.object_info.id));
                        break;
                    }
                }
            }
            rt.push_back(convexhull_array);
        }
        // ROS_INFO("time array size : %d, return convexhull array size : %d", n, int(rt.size()));
        return rt;
    }

    inline vector<ConvexHull> getConvexHull(const vector<hmg_utils::ConvexHull>& obstacles){
        /*
        input
        obstacles : list of static obstacles

        output
        rt : vector of ConvexHull
        */
        int n = obstacles.size();
        vector<ConvexHull> rt;

        for(int i=0;i<n;i++){
            vector<point> p;
            for(int j=0;j<obstacles[i].count;j++){
                p.push_back(point(obstacles[i].points[j].x,obstacles[i].points[j].y));
            }
            rt.push_back(ConvexHull(p));
        }
        return rt;
    }

    // inline vector<ConvexHull> getStaticConvexHull(const vector<hmg_utils::ConvexHull>& static_obstacles){
    //     vector<ConvexHull> rt;
    //     for(hmg_utils::ConvexHull cvh : static_obstacles){
    //         for(geometry_msgs::Point point1 : cvh.points){
    //             vector<point> p;
    //             p.push_back(point(point1.x, point1.y));
    //         }
    //         rt.push_back(ConvexHull(r));
    //     }
    //     return rt;
    // }

    void PredictionCallback(const hmg_utils::PredictionArray & msg){
        pa = msg;
        ROS_INFO("[PP] # OF PREDICTION ARRAY : %d", int(msg.predictions.size()));
    }

    void PedestrianCallback(const hmg_utils::PedestrianArray & msg){
        psa = msg;
    }

    void ObstacleCallback(const hmg_utils::ConvexHullArray & msg){
        cha = msg;
    }

    void ObjectIdCallback(const hmg_utils::ObjectId & msg){
        prv_id = msg.prv;
        cur_id = msg.cur;
    }

    void controlSet(){
        if(debug) ROS_INFO("[PP] controlSet called");

        vector<double> accel_steer = {0,0};
        double next_accel = 0;
        double next_c = 0;

        if(best_frenet_path.flag == -1){
            last_fail_count++;
            last_succeed_count = 0;
            if( motion ==2 || motion ==3 || task ==2) last_fail_count = max(last_fail_count,10);
            ROS_ERROR("[PP] No Path, Count : %d, MotionState : %d", last_fail_count, motion);
            if( motion !=2 && motion !=3 && task !=2 && last_fail_count<=2){
                /// Previous Path
                ROS_ERROR("[PP] No Path so Skip %d", last_fail_count);
                int temp_count = min({
                    last_fail_count, 2, 
                    int(best_frenet_path.accel.size()-1), 
                    int(best_frenet_path.c.size()-1)
                });
                if(temp_count < 0){
                    next_accel = 0;
                    next_c = 0;
                }
                else{
                    next_accel = best_frenet_path.accel[temp_count];
                    next_c = best_frenet_path.c[temp_count];
                }
            }else if((motion !=3 && task !=2 && last_fail_count > 100) || last_fail_count > 600 ){
                // Pure Pursuit Escape
                ROS_ERROR("[PP] No Path Pure Pursuit E-escape %d", last_fail_count);
                SLState sls = csp->transform(car.poseState);
                sls.s += fot_hp.target_s_length;
                sls.l = 0;
                PoseState next_ps = csp->sl_to_xy(sls);
                next_accel = (5.0 - sls.ds) * 0.2;
                PurePursuit pp(car.poseState.yaw, car.poseState.x, car.poseState.y, next_ps.x, next_ps.y, {0});
                next_c = pp.curvature;
                accel_steer = car.findAccelSteer(next_accel,next_c);
                ros::param::set("Ax",next_accel);
                ros::param::set("SteeringWheel",accel_steer[1]);
                ROS_WARN("Ax: %lf, SW: %lf",next_accel,accel_steer[1]);
                return;
            }else{
                /// ESTOP
                ROS_ERROR("[PP] No Path Pure Pursuit E-escape %d", last_fail_count);
                SLState sls = csp->transform(car.poseState);
                sls.s += fot_hp.target_s_length;
                sls.l = 0;
                PoseState next_ps = csp->sl_to_xy(sls);
                next_accel = max(-10.0, -1.0*last_fail_count);
                PoseState transformed = car.poseState.transform(car.poseState.getPose());
                if(transformed.vx<1e-2) next_accel = max(next_accel,-1e-6);    ////
                PurePursuit pp(car.poseState.yaw, car.poseState.x, car.poseState.y, next_ps.x, next_ps.y, {0});
                next_c = pp.curvature;
                accel_steer = car.findAccelSteer(next_accel,next_c);
                ros::param::set("Ax",next_accel);
                ros::param::set("SteeringWheel",accel_steer[1]);
                ROS_WARN("Ax: %lf, SW: %lf",next_accel,accel_steer[1]);
                return;
            }
        }else{
            /// Path Exist
            last_fail_count=0;
            last_succeed_count++;

            if(task==2 && last_succeed_count<3){
                /// Intersection ESTOP
                SLState sls = csp->transform(car.poseState);
                sls.s += fot_hp.target_s_length;
                sls.l = 0;
                PoseState next_ps = csp->sl_to_xy(sls);
                next_accel = -10.0;
                PoseState transformed = car.poseState.transform(car.poseState.getPose());
                if(transformed.vx<1e-2) next_accel = max(next_accel,-1e-6);    ////
                PurePursuit pp(car.poseState.yaw, car.poseState.x, car.poseState.y, next_ps.x, next_ps.y, {0});
                next_c = pp.curvature;
                accel_steer = car.findAccelSteer(next_accel,next_c);
                ros::param::set("Ax",next_accel);
                ros::param::set("SteeringWheel",accel_steer[1]);
                ROS_WARN("Ax: %lf, SW: %lf",next_accel,accel_steer[1]);
                return;
            }

            if(debug) ROS_INFO("[PP] yes path");
            if(debug) ROS_INFO("[PP] flag s.size accel.size c.size %d %d %d %d", best_frenet_path.flag, best_frenet_path.s.size(), best_frenet_path.accel.size(), best_frenet_path.c.size());
            next_accel = best_frenet_path.accel[1];
            next_c = best_frenet_path.c[1];
        }

        if(next_accel >= fot_hp.max_accel){
            ROS_ERROR("over accel %lf/%lf",next_accel,fot_hp.max_accel);
            next_accel = fot_hp.max_accel;
        }
        else if(next_accel <= -fot_hp.max_break){
            ROS_ERROR("over break %lf/%lf",next_accel,fot_hp.max_break);
            next_accel = -fot_hp.max_break;
        }
        if( abs(next_c) >= fot_hp.max_curvature){
            ROS_ERROR("over curvature %lf/%lf",next_c,fot_hp.max_curvature);
            next_c *= fot_hp.max_curvature/abs(next_c);
        }
        accel_steer = car.findAccelSteer(next_accel,next_c);
        //car.setPose(car.simulate(accel_steer[0], accel_steer[1], best_frenet_path.t[1] - best_frenet_path.t[0]));
        
        PoseState transformed = car.poseState.transform(car.poseState.getPose());
        if(transformed.vx<0.3 && accel_steer[0]>1e-2) accel_steer[0] = max(accel_steer[0], 0.3);    ///
        if(transformed.vx<1e-2) next_accel = max(next_accel,-1e-6);    ////
        ros::param::set("Ax",accel_steer[0]);
        
        double prev_steer = 0;
        ros::param::get("SteeringWheel",prev_steer);
        if(prev_steer + max_steer_diff < accel_steer[1]) accel_steer[1] = prev_steer + max_steer_diff;
        if(prev_steer - max_steer_diff > accel_steer[1]) accel_steer[1] = prev_steer - max_steer_diff;

        ros::param::set("SteeringWheel",accel_steer[1]);

        if(debug) ROS_WARN("Ax: %lf, SW: %lf",accel_steer[0], accel_steer[1]);
    }
    // void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //     // ROS_WARN("odometry callback called");
    //     ps.x = msg->pose.pose.position.x;
    //     ps.y = msg->pose.pose.position.y;
    //     geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    //     ps.yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
    //     ps.vx = msg->twist.twist.linear.x;
    //     ps.vy = msg->twist.twist.linear.y;
    //     ps.yawrate = msg->twist.twist.angular.z;
    // }
    // void imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    //     // ROS_WARN("imu callback called");
    //     double gamma = 1.00;
    //     ps.ax = ps.ax*(1.0-gamma) + msg->linear_acceleration.x*gamma;
    //     ps.ay = ps.ay*(1.0-gamma) + msg->linear_acceleration.y*gamma;
    // }
    void vehicleinfoCallback(const hellocm_msgs::VehicleInfo::ConstPtr& msg){
        ROS_WARN("[PP] VehicleInfo Callback");
        ps.x = msg->x[0];
        ps.y = msg->x[1];
        ps.yaw = msg->yaw;
        ps.vx = msg->v[0];
        ps.vy = msg->v[1];
        ps.yawrate = msg->yawrate;
        ps.ax = msg->a[0];
        ps.ay = msg->a[1];
    }
    
    void targetsCallback(const hmg_utils::Targets::ConstPtr& msg){
        static int count = 0;
        count++;
        if(count%2==1)return;
        time_check(PATH_PLANNING, START);
        ROS_WARN("[PP] targets callback called");
        task = msg->task;
        state = msg->state;
        motion = msg->motion;
        
        car.setPose(ps);
        PoseState transformed = car.poseState.transform(car.poseState.getPose());
        //transformed.ay *= 0.9;
        
        if(debug){
            ROS_WARN("+++Car Coordinate+++");
            ROS_WARN(" %lf %lf %lf",transformed.x, transformed.y, transformed.yaw);
            ROS_WARN(" %lf %lf %lf",transformed.vx, transformed.vy, transformed.yawrate);
            ROS_WARN(" %lf %lf",transformed.ax, transformed.ay);
        }

        // transformed.x += car.tail_length;
        transformed.vx = max(transformed.vx,1e-4);
        // transformed.vy += car.tail_length * transformed.yawrate;
        // transformed.vy = 0;
        // transformed.ax -= car.tail_length * transformed.yawrate * transformed.yawrate;

        PoseState noSlipped = transformed.transform(car.poseState.getPose(),true);
        
        if(debug){
            ROS_WARN("+++CarAxis Coordinate+++");
            ROS_WARN(" %lf %lf %lf",transformed.x, transformed.y, transformed.yaw);
            ROS_WARN(" %lf %lf %lf",transformed.vx, transformed.vy, transformed.yawrate);
            ROS_WARN(" %lf %lf",transformed.ax, transformed.ay);
        
            ROS_WARN("+++Global Coordinate+++");
            ROS_WARN(" %lf %lf %lf",car.poseState.x, car.poseState.y, car.poseState.yaw);
            ROS_WARN(" %lf %lf %lf",car.poseState.vx, car.poseState.vy, car.poseState.yawrate);
            ROS_WARN(" %lf %lf",car.poseState.ax, car.poseState.ay);
        }
        time_check(PATH_PLANNING, CHECK, "pre-processing");
        if(visual){
            prev_pos_x.push_back(car.poseState.x);
            prev_pos_y.push_back(car.poseState.y);
            if(prev_pos_x.size()>200) prev_pos_x.erase(prev_pos_x.begin(),prev_pos_x.begin()+100);
            if(prev_pos_y.size()>200) prev_pos_y.erase(prev_pos_y.begin(),prev_pos_y.begin()+100);
        }
        time_check(PATH_PLANNING, CHECK, "visual pos push");

        if(debug) ROS_INFO("targets size %d",(int)(msg->targets.size()));
        fot_ic.s_min = msg->s_min;
        if(msg->targets.empty()){
            ROS_ERROR("empty target");
            return;
        }
        best_frenet_path = FrenetPath();
        for(hmg_utils::Target target : msg->targets){
            if(debug) ROS_INFO("target segment %d", target.segment);
            csp = &(cubics[target.segment]);
            double s_length = csp->calc_s_length() - 1e-5;
            if(debug){
                ROS_WARN("csp %d : %s",target.segment,csp->get_id()[1].c_str());    
                ROS_WARN("csp from x %lf y %lf yaw %lf",csp->calc_x(1e-5),csp->calc_y(1e-5),csp->calc_yaw(1e-5));
                ROS_WARN("csp to s %lf : x %lf y %lf yaw %lf", s_length, csp->calc_x(s_length),csp->calc_y(s_length),csp->calc_yaw(s_length));
            }
            SLState sls = csp->transform(noSlipped);
            sls.ds += 0.0;
            fot_ic.s0 = sls.s;
            fot_ic.c_speed = sls.ds;
            fot_ic.c_accel = sls.dds;
            fot_ic.c_d = sls.l;
            fot_ic.c_d_d = sls.dl/sls.ds;
            //fot_ic.c_d_dd = (sls.ddl - sls.dl*sls.dds/sls.ds)/sq(sls.ds);
            double prev_steer;
            ros::param::get("SteeringWheel",prev_steer);
            fot_ic.c_d_dd = hypot(1,fot_ic.c_d_d)*hypot(1,fot_ic.c_d_d)*hypot(1,fot_ic.c_d_d);
            fot_ic.c_d_dd *= prev_steer;
            fot_ic.c_d_dd /= car.steering_ratio*(car.front_length + car.rear_length + car.understeer_gradient*sq(transformed.vx));
            ///
            if(sls.ds>1.0){
                fot_ic.c_d_dd = (fot_ic.c_d_dd*125 + sls.ddl*sls.ds - sls.dl*sls.dds)/(125+sq(sls.ds)*sls.ds);
            }
            if( abs(fot_ic.c_d_dd) > 0.9*fot_hp.max_curvature){
                ROS_ERROR("[PP] initial over curvature");
                fot_ic.c_d_dd *= 0.9*fot_hp.max_curvature/abs(fot_ic.c_d_dd);
            }
            //double prev_steer = 0;
            //ros::param::get("SteeringWheel",prev_steer);
            //fot_ic.c_d_dd = prev_steer * hypot(1, fot_ic.c_d_d)* hypot(1, fot_ic.c_d_d)* hypot(1, fot_ic.c_d_d);
            if(sls.ds<1e-2){
                ROS_ERROR("[PP] zero speed");
                fot_ic.c_d_d = 0;
                fot_ic.c_d_dd = 0;
            }
            if(debug){
                ROS_WARN("+++SL Coordinate+++");
                ROS_WARN("%+lf %lf %lf",sls.s, sls.ds, sls.dds);
                ROS_WARN("% lf %lf %lf",sls.l, sls.dl, sls.ddl);
            }

            fot_ic.flag = target.flag;
            fot_ic.target_speed = target.velocity;
            fot_ic.target_speed += 0.0;
            fot_ic.l_offset = target.l_offset * min(1.0,max(sls.ds,0.0)/1.0);
            if(debug) ROS_INFO("target l offset %lf %lf", target.l_offset, fot_ic.l_offset);
            double t0 =msg->header.stamp.toSec();

            double speed = fot_ic.c_speed*0.25 + fot_ic.target_speed*0.75;
            if(speed < 0.5){

                continue;
            }
            double dt = fot_hp.ds_length / speed;
            double maxt = t0 + fot_hp.max_s_length / speed;

            double ti = t0;
            while (ti <= t0 + 2*fot_hp.planning_t){
                fot_ic.t.push_back(ti);
                ti+=fot_hp.control_t;
            }
            // t.push_back(ti);
            // ti += dt-remainder(ti,dt);
            while (ti <= maxt){
                fot_ic.t.push_back(ti);
                ti+=dt;
            }
            auto ba = global_map.PredictionService(
                pa.predictions, fot_ic.t
            );
            auto ba_ps = global_map.PedestrianService(
                psa.predictions, fot_ic.t
            );
            ba.insert(ba.end(),ba_ps.begin(),ba_ps.end());
            dynamic_cvh_arrays = getConvexHull(ba,msg->back_cars);
            static_cvh_array = getConvexHull(cha.obstacles);
            fot_ic.dynamic_obstacles = dynamic_cvh_arrays;
            fot_ic.static_obstacles = static_cvh_array;

            if(target.flag == 0){
                // 속도 추종
                if(debug){
                    ROS_WARN("target_vel : %lf", fot_ic.target_speed);
                }
            }else if(target.flag ==1){
                // 점 추종
                if(debug){
                    ROS_WARN("target_id : %s, target_segment : %d", target.target_front.c_str(), target.segment);
                }
                vector<double> target_front_s;
                vector<double> target_front_speed;
                vector<double> target_front_accel;

                for(hmg_utils::BoxArray boxarray : ba){
                    hmg_utils::Box *target_front = nullptr;
                    hmg_utils::Box *target_back = nullptr;
                    for(hmg_utils::Box & box : boxarray.boxes){
                        if((box.object_info.id == target.target_front) && (box.segment_index == target.segment)){
                            target_front = &box;
                        }
                        if((box.object_info.id == target.target_back) && (box.segment_index == target.segment)){
                            target_back = &box;
                        }
                    }
                    if(target_front==nullptr){
                        if(target_back==nullptr){
                            ROS_ERROR("[pp] no target point");
                            //assert(false);
                            break;   /// continue??
                        }
                        fot_ic.target_point_s.push_back(target_back->s_max + target_back->object_info.length/2.0 + safe_0 + safe_1*abs(target_back->v));
                        fot_ic.target_point_speed.push_back(target_back->v);
                        fot_ic.target_point_accel.push_back(0.0);
                    }else if(target_back==nullptr){
                        fot_ic.target_point_s.push_back(target_front->s_min - target_front->object_info.length/2.0 - safe_0 - safe_1*abs(target_front->v));
                        fot_ic.target_point_speed.push_back(target_front->v);
                        fot_ic.target_point_accel.push_back(0.0);
                    }else{
                        fot_ic.target_point_s.push_back((target_front->s_min+target_back->s_max)/2);
                        fot_ic.target_point_speed.push_back((target_front->v+target_back->v)/2);
                        fot_ic.target_point_accel.push_back(0.0);
                    }
                }
            }
            else if(target.flag==2){
                if(debug){
                    ROS_WARN("target_point_s : %d", target.s);
                }
                fot_ic.target_point_s = vector<double>(fot_ic.t.size(),target.s);
                fot_ic.target_point_speed = vector<double>(fot_ic.t.size(),0);
                fot_ic.target_point_accel = vector<double>(fot_ic.t.size(),0);
            }
            time_check(PATH_PLANNING, CHECK, "target defined");
            FrenetOptimalTrajectory fot; ////
            fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp, csp);
            fot_ic.t.clear();
            fot_ic.target_point_s.clear();
            fot_ic.target_point_speed.clear();
            fot_ic.target_point_accel.clear();
            FrenetPath tfp = fot.getBestPath();
            time_check(PATH_PLANNING, CHECK, "calculate best path for a target");

            if(tfp.flag != -1){
                tfp.flag = target.flag;
                // if(target.flag==0) tfp.cf += 1e6;
                tfp.cf -= 1e6 * target.priority;

                if(best_frenet_path.flag != -1){
                    if(tfp.cf < best_frenet_path.cf){
                        // delete best_frenet_path;
                        best_frenet_path = tfp;
                    }else{
                        // delete tfp;
                    }
                }else{
                    best_frenet_path = tfp;
                }
            }
            time_check(PATH_PLANNING, CHECK, "best-of-best path compare");
        }

        if(best_frenet_path.flag != -1){
            csp = &(cubics[best_frenet_path.cspno]);
        }else{
            csp = &(cubics[msg->targets[0].segment]);
        }

        /////////////////////////////////////////////////////////////////
        // SLState sls = csp->transform(noSlipped);
        // double init_s = sls.s;
        // double init_ds = sls.ds;
        // sls.s += fot_hp.target_s_length;
        // sls.l = 0;
        // PoseState pure_pursuit_target = csp->sl_to_xy(sls);
        // double accel = (5 - init_ds) * 0.5;
        // vector<double> s;
        // for(double t = 0;t<= fot_hp.target_s_length + 1e-6; t += fot_hp.target_s_length/10.0){
        //     s.push_back(t);
        // }
        // PurePursuit pp(noSlipped.yaw, noSlipped.x, noSlipped.y, pure_pursuit_target.x, pure_pursuit_target.y, s);
        // best_frenet_path = FrenetPath(&fot_hp);
        // best_frenet_path.x = pp.x;
        // best_frenet_path.y = pp.y;
        // best_frenet_path.c = vector<double>(10, pp.curvature);
        // best_frenet_path.accel = vector<double>(10, accel);
        /////////////////////////////////////////////////////////////////

        // ///////////////////////////////////////////////////////////////////////////////////////////////
        // fot_ic.target_speed = 0;
        // double t0 =msg->header.stamp.toSec();
        // double speed = max((fot_ic.c_speed*0.25+fot_ic.target_speed*0.75), 0.1);
        // double dt = fot_hp.ds_length / speed;
        // double maxt = t0 + fot_hp.max_s_length / speed;

        
        // double ti = t0;
        // while (ti <= t0 + 2*fot_hp.planning_t){
        //     fot_ic.t.push_back(ti);
        //     ti+=fot_hp.control_t;
        // }
        // // t.push_back(ti);
        // // ti += dt-remainder(ti,dt);
        // while (ti <= maxt){
        //     fot_ic.t.push_back(ti);
        //     ti+=dt;
        // }
        // fot_ic.target_point_s = vector<double>(fot_ic.t.size(),100);
        // fot_ic.target_point_speed = vector<double>(fot_ic.t.size(),0);
        // fot_ic.target_point_accel = vector<double>(fot_ic.t.size(),0);
        // fot_ic.flag = 1;
        // fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp, csp);
        // best_frenet_path = fot.getBestPath();
        // fot_ic.t.clear();
        // fot_ic.target_point_s.clear();
        // fot_ic.target_point_speed.clear();
        // fot_ic.target_point_accel.clear();
        // /////////////////////////////////////////////////////////////////
        controlSet();
        time_check(PATH_PLANNING, CHECK, "control set");
        if(debug) ROS_WARN("[PP] after control set");
        if(visual) visualize();
        time_check(PATH_PLANNING, CHECK, "visualize");
    }
    void visualize(){
        // 1 pixel = 10cm
        int cv_h = 600, cv_w = 600;
        cv::namedWindow("local map");
        cv::Mat cv_image(cv_h,cv_w,CV_8UC3,cv::Scalar(255,255,255));
        
        // target segment
        vector<double> cv_x = csp->get_x();
        vector<double> cv_y = csp->get_y();
        for(int i =0; i<cv_x.size();i++){
            int x = (cv_x[i] - ps.x)*10 + cv_w/2;
            int y = -(cv_y[i] - ps.y)*10 + cv_h/2;
            cv::circle(cv_image, cv::Point(x,y), 1, cv::Scalar(0,255,0));
        }
        SLState tmpsl;
        for(double tmps = 0; tmps < csp->get_length(); tmps += 1.0){
            tmpsl.s = tmps;
            tmpsl.l = 0;
            PoseState tmpps = csp->sl_to_xy(tmpsl);
            int x = (tmpps.x - ps.x)*10 + cv_w/2;
            int y = -(tmpps.y - ps.y)*10 + cv_h/2;
            cv::circle(cv_image, cv::Point(x,y), 1, cv::Scalar(200,0,0));
        }

        // near segment
        for(int i = 3;i<9;i+=1){
            if(csp->get_edge()[i] == -1) continue;
            CubicSpline2D *csp_other = &cubics[csp->get_edge()[i]];
            cv_x = csp_other->get_x();
            cv_y = csp_other->get_y();
            for(int i =0; i<cv_x.size();i++){
                int x = (cv_x[i] - ps.x)*10 + cv_w/2;
                int y = -(cv_y[i] - ps.y)*10 + cv_h/2;
                cv::circle(cv_image, cv::Point(x,y), 1, cv::Scalar(0,150,0));
            }
        }
        
        if(best_frenet_path.flag != -1){
            // car
            cv::RotatedRect cv_car = cv::RotatedRect(cv::Point2f(cv_h/2, cv_w/2), cv::Size2f(43, 18), -ps.yaw * 180 / 3.141592);
            cv::Point2f cv_car_vertices[4];
            cv_car.points(cv_car_vertices);
            for (int i = 0; i < 4; i++){
                cv::line(cv_image, cv_car_vertices[i], cv_car_vertices[(i+1)%4], cv::Scalar(255,128,128), 1);
            }
            // path
            int n = best_frenet_path.x.size();
            for(int i = 0; i < n; i++){
                int x = (best_frenet_path.x[i] - ps.x)*10 + cv_w/2;
                int y = -(best_frenet_path.y[i] - ps.y)*10 + cv_h/2;
                cv::circle(cv_image, cv::Point(x,y), 1, cv::Scalar(0,int(200.0*i/n),int(200.0*i/n)), -1);
            }
        }
        else{
            // car
            cv::RotatedRect cv_car = cv::RotatedRect(cv::Point2f(cv_h/2, cv_w/2), cv::Size2f(30,10), -ps.yaw * 180 / 3.141592);
            cv::Point2f cv_car_vertices[4];
            cv_car.points(cv_car_vertices);
            for (int i = 0; i < 4; i++){
                cv::line(cv_image, cv_car_vertices[i], cv_car_vertices[(i+1)%4], cv::Scalar(0,0,255), 2);
            }
        }

        // prev position
        for(int i =0;i<prev_pos_x.size(); i++){
            int x = (prev_pos_x[i] - ps.x)*10 + cv_w/2;
            int y = -(prev_pos_y[i] - ps.y)*10 + cv_h/2;
            cv::circle(cv_image, cv::Point(x,y), 1, cv::Scalar(150,150,250), -1);
        }

        // convex hull
        int tm = dynamic_cvh_arrays.size();
        for(int t = 0; t < tm; t++){
            int n = dynamic_cvh_arrays[t].size();
            for(int i=0;i<n;i++){
                int np = dynamic_cvh_arrays[t][i].p.size();
                for(int j=0;j<np;j++){
                    int xs = (dynamic_cvh_arrays[t][i].p[j].x - ps.x)*10 + cv_w/2;
                    int ys = -(dynamic_cvh_arrays[t][i].p[j].y - ps.y)*10 + cv_h/2;
                    int xf = (dynamic_cvh_arrays[t][i].p[(j+1)%np].x - ps.x)*10 + cv_w/2;
                    int yf = -(dynamic_cvh_arrays[t][i].p[(j+1)%np].y - ps.y)*10 + cv_h/2;
                    cv::line(cv_image, cv::Point(xs,ys), cv::Point(xf,yf), cv::Scalar(int(255.0*t/tm),0,int(255.0*t/tm)), 2);
                    cv::circle(cv_image, cv::Point(xs,ys), 2, cv::Scalar(0,int(255.0*t/tm),0), -1);
                    if(t==0&&j==0) cv::putText(cv_image, dynamic_cvh_arrays[t][i].id, cv::Point(xs,ys), 1, 1, cv::Scalar(0,0,0), 1, 1);
                }
            }
        }


        for(ConvexHull cvh : static_cvh_array){
            int np = cvh.p.size();
            for(int j=0;j<np;j++){
                int xs = (cvh.p[j].x - ps.x)*10 + cv_w/2;
                int ys = -(cvh.p[j].y - ps.y)*10 + cv_h/2;
                int xf = (cvh.p[(j+1)%np].x - ps.x)*10 + cv_w/2;
                int yf = -(cvh.p[(j+1)%np].y - ps.y)*10 + cv_h/2;
                cv::line(cv_image, cv::Point(xs,ys), cv::Point(xf,yf), cv::Scalar(100,0,200), 2);
                cv::circle(cv_image, cv::Point(xs,ys), 2, cv::Scalar(0,0,0), -1);
            }
        }

        string state_str[3] = {"URGENT", "BUSY", "NORMAL"};
        string motion_str[4] = {"DRIVE", "LANE_CHANGE", "CAUTION", "STOP"};
        string flag_str[2] = {"VELOCITY", "TARGET ID"};
        string flag_str_out;
        if(best_frenet_path.flag < 0 || best_frenet_path.flag >= 2) flag_str_out = "NO PATH";
        else flag_str_out = flag_str[best_frenet_path.flag];
 
        cv::putText(cv_image, state_str[state], cv::Point(0, (int)cv_h*3/4), 1, 3, cv::Scalar(255, 255, 0), 3, 4);
        cv::putText(cv_image, motion_str[motion], cv::Point(0, (int)cv_h*10/12), 1, 3, cv::Scalar(255, 255, 0), 3, 4);
        cv::putText(cv_image, flag_str_out, cv::Point(0, (int)cv_h*11/12), 1, 3, cv::Scalar(255, 255, 0), 3, 4);

        cv::Scalar turnon(0,255,255);
        cv::Scalar turnoff(0,10,10);
        int light_indicator;
        ros::param::get("/light_indicator", light_indicator);
        if(light_indicator == 1){
            cv::circle(cv_image, cv::Point((int)cv_w*3/4, (int)cv_h*11/12), 20, turnon, -1);
            cv::circle(cv_image, cv::Point((int)cv_w*11/12, (int)cv_h*11/12), 20, turnoff, -1);
        }
        else if(light_indicator == -1){
            cv::circle(cv_image, cv::Point((int)cv_w*3/4, (int)cv_h*11/12), 20, turnoff, -1);
            cv::circle(cv_image, cv::Point((int)cv_w*11/12, (int)cv_h*11/12), 20, turnon, -1);
        }
        else{
            cv::circle(cv_image, cv::Point((int)cv_w*3/4, (int)cv_h*11/12), 20, turnoff, -1);
            cv::circle(cv_image, cv::Point((int)cv_w*11/12, (int)cv_h*11/12), 20, turnoff, -1);
        }
        
        sort(prv_id.begin(), prv_id.end());
        sort(cur_id.begin(), cur_id.end());
        cv::putText(cv_image, "previous id", cv::Point(20, 20), 1,1.5,cv::Scalar(0,0,0),1,1);
        cv::putText(cv_image, "current id", cv::Point(60, 20), 1,1.5,cv::Scalar(0,0,0),1,1);
        for(int i=0;i<prv_id.size();i++){
            cv::putText(cv_image, prv_id[i], cv::Point(20,30+i*10), 1,1.5,cv::Scalar(0,0,0),1,1);
        }
        for(int i=0;i<cur_id.size();i++){
            cv::putText(cv_image, cur_id[i], cv::Point(60, 30+i*10),1,1.5,cv::Scalar(0,0,0),1,1);
        }

        cv::imshow("local map",cv_image);


        cv::waitKey(10);
    }
    // void plan(){
    //     car.setPose(ps);
    //     SLState sls = csp->transform(car.poseState);
    //     ROS_WARN("xy to sl ed");
    //     fot_ic.s0 = sls.s;
    //     fot_ic.c_speed = sls.ds;
    //     fot_ic.c_accel = sls.dds;
    //     fot_ic.c_d = sls.l;
    //     fot_ic.c_d_d = sls.dl;
    //     fot_ic.c_d_dd = sls.ddl;
    //     ROS_WARN("%+lf %lf %lf",car.poseState.x, car.poseState.y, car.poseState.yaw);
    //     ROS_WARN("% lf %lf %lf",car.poseState.vx, car.poseState.vy, car.poseState.yawrate);
    //     ROS_WARN("% lf %lf %lf",car.poseState.ax, car.poseState.ay, car.poseState.yaw);
    //     fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp, csp);
    //     best_frenet_path = fot.getBestPath();
    // }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planning_node");
    FrenetHyperparameters fot_hp;
    ros::param::get("/max_speed", fot_hp.max_speed);
    ros::param::get("/max_accel", fot_hp.max_accel);
    ros::param::get("/max_break", fot_hp.max_break);
    ros::param::get("/max_curvature", fot_hp.max_curvature);
    ros::param::get("/max_road_width_l", fot_hp.max_road_width_l);
    ros::param::get("/max_road_width_r", fot_hp.max_road_width_r);
    ros::param::get("/d_road_w", fot_hp.d_road_w);
    ros::param::get("/dt", fot_hp.dt);
    ros::param::get("/max_ds", fot_hp.max_ds);
    ros::param::get("/ds", fot_hp.ds);
    // ros::param::get("/maxt", fot_hp.maxt);
    // ros::param::get("/mint", fot_hp.mint);
    // ros::param::get("/target_s", fot_hp.target_s);
    ros::param::get("/ds_length", fot_hp.ds_length);
    ros::param::get("/max_s_length", fot_hp.max_s_length);
    ros::param::get("/min_s_length", fot_hp.min_s_length);
    ros::param::get("/target_s_length", fot_hp.target_s_length);
    ros::param::get("/control_t", fot_hp.control_t);
    ros::param::get("/planning_t", fot_hp.planning_t);
    ros::param::get("/d_t_s", fot_hp.d_t_s);
    ros::param::get("/n_s_sample", fot_hp.n_s_sample);
    ros::param::get("/obstacle_clearance", fot_hp.obstacle_clearance);
    ros::param::get("/kend", fot_hp.kend);
    ros::param::get("/kd", fot_hp.kd);
    ros::param::get("/kv", fot_hp.kv);
    ros::param::get("/ka", fot_hp.ka);
    ros::param::get("/kj", fot_hp.kj);
    ros::param::get("/kt", fot_hp.kt);
    ros::param::get("/ko", fot_hp.ko);
    ros::param::get("/klat", fot_hp.klat);
    ros::param::get("/klon", fot_hp.klon);
    ros::param::get("/max_steer_diff", max_steer_diff);
    ros::param::get("/safe_0", safe_0);
    ros::param::get("/safe_1", safe_1);
    ros::param::get("/pp_debug",debug);
    ros::param::get("/pp_visual",visual);
    ros::param::get("/pp_time_debug",time_debug);
    fot_hp.debug = debug;
    PathPlanningNode path_planning_node(fot_hp);
    ros::Duration(5.0).sleep();
    // ros::Rate r(200);
    // while (ros::ok())
    //     {
    //     ros::spinOnce();
    //     r.sleep();
    // }
    ros::spin();
    
    return 0;
}
