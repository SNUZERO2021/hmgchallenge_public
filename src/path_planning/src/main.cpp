#include "frenet_optimal_trajectory.h"
#include "frenet_path.h"
#include "cpp_struct.h"
#include "ros/ros.h"
#include "ros/console.h"
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "utils.h"
#include <ctime>

#include <vector>
#include <tuple>
#include <cmath>

#include <iostream>
#include <string>

#include <stdlib.h>
#include <time.h> 

using namespace std;
//using namespace cv;

CubicSpline2D* get_spline(string path){
    vector<double> x;
    vector<double> y;
    const char delimiter =  ' ';
    string in_line;
    ifstream in(path);
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter))tokens.push_back(token);
        if(tokens.size()<2) continue;
        x.push_back(stod(tokens[0]));
        y.push_back(stod(tokens[1]));
    }
    CubicSpline2D* csp = new CubicSpline2D(x,y); 
    return csp;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "frenet_optimal_trejectory_main");
    srand (time(NULL));

    double wx [6] = {0, 4, 8, 12, 16, 45};
    double wy [6] = {0, 1.8, 2.7, 1.8, 0, -2.0};
    double o_llx[3] = {25.0, 40.0, 57.0};
    double o_lly[3] = {0.4, -2.2, -2.1};
    double o_urx[3] = {30.0, 45.0, 62.0};
    double o_ury[3] = {2.2, -0.4, -0.3};
    
    FrenetInitialConditions fot_ic = {
        1,  // flag
        0.0,    // s_0
        0.0,    // c_speed
        0.0,    // c_accel
        0.0,    // c_d
        0.0,    // c_d_d
        0.0,    // c_d_dd
        3.0,    // target_speed
        new vector<double>,    // target_point_s
        new vector<double>,    // target_point_speed
        new vector<double>,    // target_point_accel
        wx, // wx
        wy, // wy
        6,  // nw
        o_llx,  // o_llx
        o_lly,  // o_lly
        o_urx,  // o_urx
        o_ury,  // o_ury
        0   // no
    };

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
    Car car;
    ros::param::get("/steering_ratio", car.steering_ratio);
    ros::param::get("/understeer_gradient", car.understeer_gradient);

    //working
    int fail_count = 0;
    vector<double> x,y;
    x.assign(fot_ic.wx, fot_ic.wx + fot_ic.nw);
    y.assign(fot_ic.wy, fot_ic.wy + fot_ic.nw);
    CubicSpline2D *csp = new CubicSpline2D(x, y);
    //--debug
    stringstream save_path;
    //save_path << ros::package::getPath("global_map_gen") << "/spline_points/G2G1F2_2L2.txt";
    save_path << ros::package::getPath("prediction_tracker") << "/segment/F2G1G2_5r4.txt";
    string string_path = save_path.str();
    csp = get_spline(string_path);  
    //debug--
    PoseState ps_init;
    ps_init.x = csp->calc_x(1.2);
    ps_init.y = csp->calc_y(1.2) + 0.1;
    ps_init.vy = 0.002;
    ps_init.ay = 1.0;
    ps_init.yaw = csp->calc_yaw(1.2) + 0.1;
    ps_init.yawrate = 0.1;
    // ROS_WARN("ps_init %lf %lf %lf",ps_init.x, ps_init.y, ps_init.yaw);
    // ps_init.x = 29.19;
    // ps_init.y = -230.06;
    // ps_init.yaw = 0.9986;
    // ps_init.vx = -0.0042;
    // ps_init.yawrate = 0.00004;
    // ps_init.x = 30;
    // ps_init.y = 0;
    // ps_init.vx = -2;
    // ps_init.vy = 0.3;
    // ps_init.yaw = 0.2;
    car.setPose(ps_init);
    
    while(1){

        //noise;
        car.poseState.x += 0.1*(0.5-(double)rand()/(double)RAND_MAX);
        car.poseState.y += 0.1*(0.5-(double)rand()/(double)RAND_MAX);
        car.poseState.yaw += 0.05*(0.5-(double)rand()/(double)RAND_MAX);
        car.poseState.vx += 0.005*(0.5-(double)rand()/(double)RAND_MAX);
        car.poseState.vy += 0.005*(0.5-(double)rand()/(double)RAND_MAX);
        car.poseState.yawrate += 0.1*(0.5-(double)rand()/(double)RAND_MAX);
        car.poseState.ax += 0.1*(0.5-(double)rand()/(double)RAND_MAX);
        car.poseState.ay += 0.1*(0.5-(double)rand()/(double)RAND_MAX);
        car.pose=car.poseState.getPose();

        clock_t startTime = clock();

        PoseState transformed = car.poseState.transform(car.poseState.getPose());
        ROS_WARN("+++Car Coordinate+++");
        ROS_WARN(" %lf %lf %lf",transformed.x, transformed.y, transformed.yaw);
        ROS_WARN(" %lf %lf %lf",transformed.vx, transformed.vy, transformed.yawrate);
        ROS_WARN(" %lf %lf",transformed.ax, transformed.ay);
        transformed.vx=max(transformed.vx,1e-2);
        transformed.vy = 0;
        PoseState noSlipped = transformed.transform(car.poseState.getPose(),true);
        SLState sls = csp->transform(noSlipped);
        fot_ic.s0 = sls.s;
        fot_ic.c_speed = sls.ds;
        fot_ic.c_accel = sls.dds;
        fot_ic.c_d = sls.l;
        fot_ic.c_d_d = sls.dl/sls.ds;
        fot_ic.c_d_dd = (sls.ddl - sls.dl*sls.dds/sls.ds)/sq(sls.ds);
        if(sls.ds<1e-2){
            ROS_ERROR("zero speed");
            fot_ic.c_d_d = 0;
            fot_ic.c_d_dd = 0;
        }
        ROS_WARN("+++Global Coordinate+++");
        ROS_WARN(" %lf %lf %lf",car.poseState.x, car.poseState.y, car.poseState.yaw);
        ROS_WARN(" % lf %lf %lf",car.poseState.vx, car.poseState.vy, car.poseState.yawrate);
        ROS_WARN(" % lf %lf",car.poseState.ax, car.poseState.ay);
        // ROS_WARN("%+lf %lf %lf",noSlipped.x, noSlipped.y, noSlipped.yaw);
        // ROS_WARN("% lf %lf %lf",noSlipped.vx, noSlipped.vy, noSlipped.yawrate);
        // ROS_WARN("% lf %lf",noSlipped.ax, noSlipped.ay);
        ROS_WARN("+++SL Coordinate+++");
        ROS_WARN("%+lf %lf %lf",sls.s, sls.ds, sls.dds);
        ROS_WARN("% lf %lf %lf",sls.l, sls.dl, sls.ddl);

        //FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp);
        FrenetOptimalTrajectory fot = FrenetOptimalTrajectory(&fot_ic, &fot_hp, csp);
        FrenetPath* best_frenet_path = fot.getBestPath();
        //CubicSpline2D* csp = fot.getcsp();
        vector<double> accel_steer = {0, 0};

        clock_t endTime = clock();
        clock_t elapsed = endTime - startTime;
        double timeInSecond = (double)elapsed / CLOCKS_PER_SEC;
        ROS_WARN("Plan_Time : %lf",timeInSecond);

        if (best_frenet_path) {
            cout << "Success\n";
            fail_count=0;
        }
        else{
            cout << "Failure\n";
            fail_count++;
            car.setPose(car.simulate(accel_steer[0], accel_steer[1], fot_hp.planning_t));
            ROS_WARN("fail count : %d",fail_count);
            //break;
            //continue;
        }

        double t_s = csp->calc_s_length();
        
        PoseState ps = car.poseState;
       // 1 pixel = 10cm
        int cv_h = 600, cv_w = 600;
        cv::namedWindow("local map");
		cv::Mat cv_image(cv_h,cv_w,CV_8UC3,cv::Scalar(0,0,0));
        
        // target segment
        vector<double> cv_x = csp->get_x();
        vector<double> cv_y = csp->get_y();
        for(int i =0; i<cv_x.size();i++){
            int x = (cv_x[i] - ps.x)*10 + cv_w/2;
            int y = -(cv_y[i] - ps.y)*10 + cv_h/2;
            cv::circle(cv_image, cv::Point(x,y), 3, cv::Scalar(0,255,0));
        }

        if(best_frenet_path != nullptr){
            // car
            cv::RotatedRect cv_car = cv::RotatedRect(cv::Point2f(cv_h/2, cv_w/2), cv::Size2f(43, 18), -ps.yaw * 180 / 3.141592);
            cv::Point2f cv_car_vertices[4];
            cv_car.points(cv_car_vertices);
            for (int i = 0; i < 4; i++){
                cv::line(cv_image, cv_car_vertices[i], cv_car_vertices[(i+1)%4], cv::Scalar(255,128,128), 1);
            }
            // path
            for(int i = 0; i < best_frenet_path->x.size();i++){
                int x = (best_frenet_path->x[i] - ps.x)*10 + cv_w/2;
                int y = -(best_frenet_path->y[i] - ps.y)*10 + cv_h/2;
                cv::circle(cv_image, cv::Point(x,y), 5, cv::Scalar(0,200,200), -1);
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

        cv::imshow("local map",cv_image);
        cv::waitKey(20);

        if(!best_frenet_path)continue;
        if(abs(t_s - best_frenet_path->s.front()) < 10)
            break;
        
        // fot_ic.s0 = best_frenet_path->s[1];
        // fot_ic.c_speed = best_frenet_path->s_d[1];
        // fot_ic.c_accel = best_frenet_path->s_dd[1];
        // fot_ic.c_d = best_frenet_path->d[1];
        // fot_ic.c_d_d = best_frenet_path->d_d[1];
        // fot_ic.c_d_dd = best_frenet_path->d_dd[1];

        startTime = clock();
        
        for(int control_count=0; best_frenet_path->t[control_count]<fot_hp.planning_t; control_count++ ){
            double next_accel = best_frenet_path->accel[control_count+1];
            double next_c = best_frenet_path->c[control_count+1];
            ROS_WARN("c0:%lf c1:%lf",best_frenet_path->c[control_count],next_c);
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
            ROS_WARN("Ax: %lf, SW: %lf",accel_steer[0], accel_steer[1]);
            car.setPose(car.simulate(accel_steer[0], accel_steer[1], best_frenet_path->t[control_count+1] - best_frenet_path->t[control_count]));
        }
        ROS_WARN("%lf",best_frenet_path->s_d.back());
        endTime = clock();
        elapsed = endTime - startTime;
        timeInSecond = (double)elapsed / CLOCKS_PER_SEC;
        ROS_WARN("Control_Time : %lf",timeInSecond);
    }
    cv::destroyAllWindows();
    
    return 0;
}
