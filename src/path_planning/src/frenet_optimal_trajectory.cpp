#include <iostream>
#include <chrono>
#include <vector>
#include "math.h"
#include "frenet_optimal_trajectory.h"
#include "GlobalMap.h"
#include "quintic_polynomial.h"
#include "quartic_polynomial.h"
#include "utils.h"
#include "ros/console.h"

using namespace std;

FrenetOptimalTrajectory::FrenetOptimalTrajectory(){}

FrenetOptimalTrajectory::FrenetOptimalTrajectory(FrenetInitialConditions *fot_ic_, FrenetHyperparameters *fot_hp_, CubicSpline2D *csp_) {
    // auto start = chrono::high_resolution_clock::now();
    if(fot_hp_->debug){
        ROS_WARN("[FOT] inited");
    }
    fot_ic = fot_ic_;
    fot_hp = fot_hp_;
    csp = csp_;
    // x.assign(fot_ic->wx, fot_ic->wx + fot_ic->nw);
    // y.assign(fot_ic->wy, fot_ic->wy + fot_ic->nw);
    // setObstacles();
    dynamic_obstacles = fot_ic->dynamic_obstacles;
    static_obstacles = fot_ic->static_obstacles;
    best_frenet_path = FrenetPath();

    // if (x.size() < 2) {
    //     return;
    // }

    //csp = new CubicSpline2D(x, y);
    
    FrenetPath::fail_speed_count = 0;
    FrenetPath::fail_accel_count = 0;
    FrenetPath::fail_break_count = 0;
    FrenetPath::fail_curvature_count = 0;   
    FrenetPath::fail_obstacle_count = 0;
    // FrenetPath::time0 = 0.0;
    // FrenetPath::time1 = 0.0;
    // FrenetPath::time2 = 0.0;s
    // FrenetPath::time3 = 0.0;
    // FrenetPath::time4 = 0.0;
    // FrenetPath::time5 = 0.0;

    time_check(PATH_PLANNING, CHECK, "FOT pre-proccess");
    calc_frenet_paths();
    time_check(PATH_PLANNING, CHECK, "FOT calc frenet path");
    if(fot_hp->debug) ROS_WARN("PP fail count speed %d, accel %d, break %d, curvature %d, obstacle %d",
        FrenetPath::fail_speed_count,FrenetPath::fail_accel_count,FrenetPath::fail_break_count,FrenetPath::fail_curvature_count,FrenetPath::fail_obstacle_count);
    double mincost = INFINITY;
    int best_index = -1;
    for (int i = 0;i< frenet_paths.size();i++) {
        FrenetPath & fp = frenet_paths[i];
        if (fp.cf <= mincost) {
            mincost = fp.cf;
            best_index = i;
            //ROS_ERROR("target ds :%lf, total cost : %lf", fp->s_d.back(), fp->cf);
            //ROS_ERROR("lat cost : %lf, longi cost : %lf, time cost : %lf, obs cost : %lf", fot_hp->klat*fp->c_lateral, fot_hp->klon*fp->c_longitudinal, fot_hp->kt*fp->c_time_taken, fot_hp->ko*fp->c_inv_dist_to_obstacles);
        }
    }
    if(best_index != -1) best_frenet_path = frenet_paths[best_index];
    if(best_frenet_path.flag == -1){
        ROS_ERROR("[FOT]No Path Error");
    }else{
        if(fot_hp->debug) ROS_WARN("[FOT]Yes Path Not Error");
        if(fot_hp->debug) ROS_WARN("target s:%lf l:%lf yaw:%lf ds:%lf dl: %lf t:%lf", best_frenet_path.s.back(), best_frenet_path.d.back(), best_frenet_path.yaw.back(), best_frenet_path.s_d.back(), best_frenet_path.d_d.back(), best_frenet_path.t.back());
        // if(best_frenet_path->s.back() > csp->calc_s_length()){
        //     ROS_ERROR("%lf %lf %lf %lf", csp->calc_x(best_frenet_path->s.back()), best_frenet_path->x.back(),best_frenet_path->accel.back(),best_frenet_path->c.back());
        // }
    }
    // auto end = chrono::high_resolution_clock::now();
    // double run_time = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
    // run_time *= 1e-9;
    // cout << "Planning runtime " << run_time << endl;
    // cout << "time 012345 " << FrenetPath::time0 << " " << FrenetPath::time1 << " " << FrenetPath::time2
    //     << " " << FrenetPath::time3 << " " << FrenetPath::time4 << " " << FrenetPath::time5 << endl;
    time_check(PATH_PLANNING, CHECK, "FOT complete");
}

FrenetOptimalTrajectory::~FrenetOptimalTrajectory() {
}

FrenetOptimalTrajectory& FrenetOptimalTrajectory::operator=(const FrenetOptimalTrajectory& o){
    fot_ic = o.fot_ic;
    fot_hp = o.fot_hp;
    best_frenet_path = o.best_frenet_path;
    csp = o.csp;
    // obstacles = o.obstacles;
    dynamic_obstacles = o.dynamic_obstacles;
    static_obstacles = o.static_obstacles;
    x = o.x;
    y = o.y;
    frenet_paths = o.frenet_paths;
}

CubicSpline2D* FrenetOptimalTrajectory::getcsp(){
    return csp;
}

FrenetPath FrenetOptimalTrajectory::getBestPath() {
    return best_frenet_path;
}

// void FrenetOptimalTrajectory::setObstacles() {
//     vector<double> llx(fot_ic->o_llx, fot_ic->o_llx + fot_ic->no);
//     vector<double> lly(fot_ic->o_lly, fot_ic->o_lly + fot_ic->no);
//     vector<double> urx(fot_ic->o_urx, fot_ic->o_urx + fot_ic->no);
//     vector<double> ury(fot_ic->o_ury, fot_ic->o_ury + fot_ic->no);

//     for (int i = 0; i < fot_ic->no; i++) {
//         addObstacle(Vector2f(llx[i], lly[i]), Vector2f(urx[i], ury[i]));
//     }
// }

// void FrenetOptimalTrajectory::addObstacle(Vector2f first_point, Vector2f second_point) {
//     obstacles.push_back(Obstacle(std::move(first_point), std::move(second_point), fot_hp->obstacle_clearance));
// }

void FrenetOptimalTrajectory::calc_frenet_paths() {
    vector<double> t;
    double lateral_deviation, lateral_velocity, lateral_acceleration, lateral_jerk;
    double longitudinal_acceleration, longitudinal_jerk;
    FrenetPath tfp;
    int num_paths = 0;
    int num_viable_paths = 0;
    double valid_path_time = 0;
    double speed = max((fot_ic->c_speed*0.25+fot_ic->target_speed*0.75), 0.1);
    double mint = fot_hp->min_s_length /speed;
    for(int i=0; i<fot_ic->t.size(); i++){
        double ti = fot_ic->t[i] - fot_ic->t.front();
        t.push_back(ti);
        if(ti<mint){
            continue;
        }
        vector<FrenetPath1D> longitudinal_paths;
        if(fot_ic->flag == 0){
            // 속도 유지
            double target_speed = fot_ic->target_speed;
            double cos_rel = 1.0/hypot(1,fot_ic->c_d_d);
            double possible_max_speed =
                fot_ic->c_speed
                +(
                    0.9*2/3*( fot_hp->max_accel - sq(fot_ic->c_speed)*fot_ic->c_d_d*fot_ic->c_d_dd*cos_rel )*cos_rel
                    + 1/6*fot_ic->c_accel
                )*ti;
            possible_max_speed = min(possible_max_speed,0.9*fot_hp->max_speed*cos_rel);
            double possible_min_speed =
                fot_ic->c_speed
                +(
                    0.9*2/3*( -fot_hp->max_break - sq(fot_ic->c_speed)*fot_ic->c_d_d*fot_ic->c_d_dd*cos_rel)*cos_rel
                    + 1/6*fot_ic->c_accel
                )*ti;
            possible_min_speed = max(possible_min_speed,1e-6);
            double d_t_s = fot_hp->d_t_s * target_speed/10;
            if(target_speed > possible_max_speed){
                target_speed = possible_max_speed;
                d_t_s = fot_hp->d_t_s * target_speed/10;
                target_speed -= d_t_s * fot_hp->n_s_sample;
            }
            else if(target_speed < possible_min_speed){
                target_speed = possible_min_speed;
                d_t_s = fot_hp->d_t_s * target_speed/10;
                target_speed += d_t_s * fot_hp->n_s_sample;
            }
            //ROS_WARN("target speed, possible_max, possible_min, final %lf %lf %lf %lf",fot_ic->target_speed,possible_max_speed,possible_min_speed,target_speed);
            longitudinal_paths = quartic_paths_1D(
                t, fot_ic->s0, fot_ic->c_speed, fot_ic->c_accel, 
                target_speed - d_t_s*fot_hp->n_s_sample, target_speed + d_t_s*fot_hp->n_s_sample, d_t_s, 
                fot_ic->target_speed, 0);
        }else{
            // 점 추종
            if(i>=fot_ic->target_point_s.size()) break; ///
            double target_point = min(fot_ic->target_point_s[i],fot_ic->s_min);
            //double target_point = fot_ic->s0 + fot_ic->target_speed*ti; //일단 테스트 용

            ///
            longitudinal_paths = quintic_paths_1D(
                t, fot_ic->s0, fot_ic->c_speed, fot_ic->c_accel, 
                target_point - fot_hp->max_ds, target_point + fot_hp->ds, fot_hp->ds, 
                target_point, fot_ic->target_point_speed[i], fot_ic->target_point_accel[i]);
        }

        for(FrenetPath1D & longitudinal_path : longitudinal_paths){
            double s_end = longitudinal_path.y.back()-longitudinal_path.y.front();
            double target_d = fot_ic->l_offset;
            double max_d = fot_ic->c_d + 0.25*fot_ic->c_d_d*s_end + 0.9*0.125*fot_hp->max_curvature*s_end*s_end;
            double min_d = fot_ic->c_d + 0.25*fot_ic->c_d_d*s_end - 0.9*0.125*fot_hp->max_curvature*s_end*s_end;
            if(target_d > max_d){
                target_d = max_d;
            }else if(target_d < min_d){
                target_d = min_d;
            }
            double max_road_width_r = fot_hp->max_road_width_r;
            double max_road_width_l = fot_hp->max_road_width_l;
            auto edge = csp->get_edge();
            if(edge[LEFT_STRAIGHT]==-1 && edge[LEFT_LEFT]==-1 && edge[LEFT_RIGHT]==-1){
                max_road_width_l = fot_hp->d_road_w;
            }
            if(edge[RIGHT_STRAIGHT]==-1 && edge[RIGHT_LEFT]==-1 && edge[RIGHT_RIGHT]==-1){
                max_road_width_r = fot_hp->d_road_w;
            }
            vector<FrenetPath1D> lateral_paths = quintic_paths_1D(
                longitudinal_path.y, fot_ic->c_d, fot_ic->c_d_d, fot_ic->c_d_dd, 
                target_d - max_road_width_r, target_d + max_road_width_l, fot_hp->d_road_w, 
                fot_ic->l_offset, 0, 0);
            for(FrenetPath1D & lateral_path : lateral_paths){
                tfp = FrenetPath(fot_hp);
                tfp.flag = fot_ic->flag;
                tfp.t.assign(t.begin(),t.end());
                tfp.d.assign(lateral_path.y.begin(),lateral_path.y.end());
                // for(int i=0; i<lateral_path.y.size(); i++){
                //     if( lateral_path.y[i] > 6000){
                //         printf("y %lf i %d y.size %d \n",lateral_path.y[i],i,lateral_path.y.size());
                //         cout << endl;
                //         assert(false);
                //     }
                // }
                tfp.d_d.assign(lateral_path.y_d.begin(),lateral_path.y_d.end());
                tfp.d_dd.assign(lateral_path.y_dd.begin(),lateral_path.y_dd.end());
                tfp.d_ddd.assign(lateral_path.y_ddd.begin(),lateral_path.y_ddd.end());
                tfp.s.assign(longitudinal_path.y.begin(),longitudinal_path.y.end());
                tfp.s_d.assign(longitudinal_path.y_d.begin(),longitudinal_path.y_d.end());
                tfp.s_dd.assign(longitudinal_path.y_dd.begin(),longitudinal_path.y_dd.end());
                tfp.s_ddd.assign(longitudinal_path.y_ddd.begin(),longitudinal_path.y_ddd.end());
                tfp.cspno = csp->get_index();

                num_paths++;
                bool success = tfp.to_global_path(csp);
                if (!success) {
                    continue;
                }
                bool valid_path = tfp.is_valid_path(dynamic_obstacles, static_obstacles);
                if (!valid_path) {
                    continue;
                }
                num_viable_paths++;
                
                // if(num_viable_paths%10==0){
                //     ROS_WARN("accel: %lf, c: %lf",tfp.accel[1], tfp.c[1]);
                // }
                tfp.c_lateral_deviation = lateral_path.c_deviation;
                tfp.c_lateral_velocity = lateral_path.c_velocity;
                tfp.c_lateral_acceleration = lateral_path.c_accel;
                tfp.c_lateral_jerk = lateral_path.c_jerk;
                tfp.c_lateral_end = lateral_path.c_end;
                tfp.c_lateral = 
                    fot_hp->kd * tfp.c_lateral_deviation 
                    + fot_hp->kv * tfp.c_lateral_velocity 
                    + fot_hp->ka * tfp.c_lateral_acceleration 
                    + fot_hp->kj * tfp.c_lateral_jerk
                    + fot_hp->kend * tfp.c_lateral_end *0;
                tfp.c_longitudinal_acceleration = lateral_path.c_accel;
                tfp.c_longitudinal_jerk = longitudinal_path.c_jerk;
                tfp.c_longitudinal_end = longitudinal_path.c_end;
                tfp.c_longitudinal = 
                    fot_hp->ka * tfp.c_longitudinal_acceleration *0
                    + fot_hp->kj * tfp.c_longitudinal_jerk
                    + fot_hp->kend * tfp.c_longitudinal_end;
                //tfp.c_time_taken = sq(ti + target_t*target_t/ti);
                double s_length = tfp.s.back()-tfp.s.front();
                tfp.c_time_taken = sq(s_length + sq(fot_hp->target_s_length)/s_length);
                //tfp.c_inv_dist_to_obstacles = sq(tfp.inverse_distance_to_obstacles(dynamic_obstacles, static_obstacles));

                tfp.cf = fot_hp->klat*tfp.c_lateral + fot_hp->klon*tfp.c_longitudinal + fot_hp->kt*tfp.c_time_taken + fot_hp->ko*tfp.c_inv_dist_to_obstacles;
                frenet_paths.push_back(tfp);
                // delete lateral_path;
            }
            // delete longitudinal_path;
        }    
    }
}


vector<FrenetPath1D> FrenetOptimalTrajectory::quartic_paths_1D(vector<double> t, double y0, double dy0, double ddy0, double min_v, double max_v, double step_v, double target_v, double dvf){
    vector<FrenetPath1D> fp1s;
    // for(double tv = min_v; tv<=max_v; tv+=step_v){
    //     QuarticPolynomial qp = QuarticPolynomial(y0, dy0, ddy0, tv, dvf, t.back());
    //     FrenetPath1D* fp1 = new FrenetPath1D();
    //     fp1.t.assign(t.begin(),t.end());
    //     for (double tp : t) {
    //         fp1.y.push_back(qp.calc_point(tp));
    //         fp1.y_d.push_back(qp.calc_first_derivative(tp));
    //         fp1.y_dd.push_back(qp.calc_second_derivative(tp));
    //         fp1.y_ddd.push_back(qp.calc_third_derivative(tp));
    //         fp1.c_jerk += sq(qp.calc_third_derivative(tp));
    //     }
    //     fp1.c_end += sq( fp1.y_d.back() - target_v );
    //     fp1s.push_back(fp1);
    // }
    double center_v = (min_v+max_v)/2;
    for(double tv = center_v; tv<=max_v; tv+=step_v){
        QuarticPolynomial qp = QuarticPolynomial(y0, dy0, ddy0, tv, dvf, t.back()-t.front());
        FrenetPath1D fp1 = FrenetPath1D();
        fp1.t.assign(t.begin(),t.end());
        for(int it=0; it<(int)(t.size()); it++){
            double tp = t[it]-t.front();
            double dt_t = 0;
            if(it>0) dt_t = (t[it]-t[it-1])/(t.back()-t.front());
            fp1.y.push_back(qp.calc_point(tp));
            fp1.y_d.push_back(qp.calc_first_derivative(tp));
            fp1.y_dd.push_back(qp.calc_second_derivative(tp));
            fp1.y_ddd.push_back(qp.calc_third_derivative(tp));
            //fp1.c_accel += dt_t*sq(qp.calc_second_derivative(tp));
            fp1.c_accel = max(fp1.c_accel,sq(qp.calc_second_derivative(tp)));
            fp1.c_jerk += dt_t*sq(qp.calc_third_derivative(tp));
        }
        fp1.c_end = sq( fp1.y_d.back() - target_v );
        fp1s.push_back(fp1);
    }
    for(double tv = center_v-step_v; tv>=min_v; tv-=step_v){
        QuarticPolynomial qp = QuarticPolynomial(y0, dy0, ddy0, tv, dvf, t.back()-t.front());
        FrenetPath1D fp1 = FrenetPath1D();
        fp1.t.assign(t.begin(),t.end());
        for(int it=0; it<(int)(t.size()); it++){
            double tp = t[it]-t.front();
            double dt_t = 0;
            if(it>0) dt_t = (t[it]-t[it-1])/(t.back()-t.front());
            fp1.y.push_back(qp.calc_point(tp));
            fp1.y_d.push_back(qp.calc_first_derivative(tp));
            fp1.y_dd.push_back(qp.calc_second_derivative(tp));
            fp1.y_ddd.push_back(qp.calc_third_derivative(tp));
            //fp1.c_accel += dt_t*sq(qp.calc_second_derivative(tp));
            fp1.c_accel = max(fp1.c_accel,sq(qp.calc_second_derivative(tp)));
            fp1.c_jerk += dt_t*sq(qp.calc_third_derivative(tp));
        }
        fp1.c_end = sq( fp1.y_d.back() - target_v );
        fp1s.push_back(fp1);
    }
    return fp1s;
}
vector<FrenetPath1D> FrenetOptimalTrajectory::quintic_paths_1D(vector<double> t, double y0, double dy0, double ddy0, double min_y, double max_y, double step_y, double target_y, double dyf, double ddyf){
    // printf("y0 %lf dy0 %lf ddy0 %lf min_y %lf max_y %lf step_y %lf target_y %lf dyf %lf ddyf %lf \n",y0,dy0,ddy0,min_y,max_y,step_y,target_y,dyf,ddyf);
    // for(double it:t){
    //     printf("%lf ",it);
    // }
    // cout<< endl;
    
    vector<FrenetPath1D> fp1s;
    // for(double ty = min_y; ty<=max_y; ty+=step_y){
    //     QuinticPolynomial qp = QuinticPolynomial(y0, dy0, ddy0, ty, dyf, ddyf, t.back());
    //     FrenetPath1D* fp1 = new FrenetPath1D(); 
    //     fp1.t.assign(t.begin(),t.end());
    //     for (double tp : t) {
    //         fp1.y.push_back(qp.calc_point(tp));
    //         fp1.y_d.push_back(qp.calc_first_derivative(tp));
    //         fp1.y_dd.push_back(qp.calc_second_derivative(tp));
    //         fp1.y_ddd.push_back(qp.calc_third_derivative(tp));
    //         fp1.c_deviation += sq(qp.calc_point(tp));
    //         fp1.c_velocity += sq(qp.calc_first_derivative(tp));
    //         fp1.c_accel += sq(qp.calc_second_derivative(tp));
    //         fp1.c_jerk += sq(qp.calc_third_derivative(tp));
    //     }
    //     fp1.c_end += sq( fp1.y.back() - target_y );
    //     fp1s.push_back(fp1);
    // }
    double center_y = (min_y+max_y)/2;
    for(double ty = center_y; ty<=max_y; ty+=step_y){
        QuinticPolynomial qp = QuinticPolynomial(y0, dy0, ddy0, ty, dyf, ddyf, t.back()-t.front());
        FrenetPath1D fp1 = FrenetPath1D(); 
        fp1.t.assign(t.begin(),t.end());
        for(int it=0; it<(int)(t.size()); it++){
            double tp = t[it]-t.front();
            double dt_t = 0;
            if(it>0) dt_t = (t[it]-t[it-1])/(t.back()-t.front());
            fp1.y.push_back(qp.calc_point(tp));
            fp1.y_d.push_back(qp.calc_first_derivative(tp));
            // if(qp.calc_first_derivative(tp) > 12) {
            //     ROS_INFO("y %.3lf, dy %.3lf ddy %.3lf, ty %.3lf, dyf %.3lf ddyf %.3lf tf %.3lf",y0, dy0, ddy0, ty, dyf, ddyf, t.back()-t.front());
            //     ROS_INFO("a0 %.3lf a1 %.3lf a2 %.3lf a3 %.3lf a4 %.3lf a5 %.3lf ", qp.a0, qp.a1, qp.a2, qp.a3, qp.a4, qp.a5);
            // }
            fp1.y_dd.push_back(qp.calc_second_derivative(tp));
            fp1.y_ddd.push_back(qp.calc_third_derivative(tp));
            fp1.c_deviation += dt_t*sq(qp.calc_point(tp));
            // fp1.c_velocity += dt_t*sq(qp.calc_first_derivative(tp));
            // fp1.c_accel += dt_t*sq(qp.calc_second_derivative(tp));
            // if(qp.calc_point(tp) * qp.calc_first_derivative(tp) > 1e-6){
            //     fp1.c_deviation += dt_t*sq(27.0/256.0*pow(abs(qp.calc_first_derivative(tp)*t.back() +4.0*qp.calc_point(tp)), 4.0)/pow(abs(qp.calc_first_derivative(tp)*t.back() +3.0*qp.calc_point(tp)), 3.0));    
            // }else{
            //     fp1.c_deviation += dt_t*sq(qp.calc_point(tp));
            // }
            fp1.c_velocity = max(fp1.c_velocity,sq(qp.calc_first_derivative(tp)));
            fp1.c_accel = max(fp1.c_accel,sq(qp.calc_second_derivative(tp)));
            fp1.c_jerk += dt_t*sq(qp.calc_third_derivative(tp)); 
        }
        fp1.c_end = sq( fp1.y.back() - target_y );
        fp1s.push_back(fp1);
    }
    for(double ty = center_y-step_y; ty>=min_y; ty-=step_y){
        QuinticPolynomial qp = QuinticPolynomial(y0, dy0, ddy0, ty, dyf, ddyf, t.back()-t.front());
        FrenetPath1D fp1 = FrenetPath1D(); 
        fp1.t.assign(t.begin(),t.end());
        for(int it=0; it<(int)(t.size()); it++){
            double tp = t[it]-t.front();
            double dt_t = 0;
            if(it>0) dt_t = (t[it]-t[it-1])/(t.back()-t.front());
            fp1.y.push_back(qp.calc_point(tp));
            fp1.y_d.push_back(qp.calc_first_derivative(tp));
            // if(qp.calc_first_derivative(tp) > 12) {
            //     ROS_INFO("down y %.3lf, dy %.3lf ddy %.3lf, ty %.3lf, dyf %.3lf ddyf %.3lf tf %.3lf",y0, dy0, ddy0, ty, dyf, ddyf, t.back()-t.front());
            //     ROS_INFO("a0 %.3lf a1 %.3lf a2 %.3lf a3 %.3lf a4 %.3lf a5 %.3lf ", qp.a0, qp.a1, qp.a2, qp.a3, qp.a4, qp.a5);
            // }
            fp1.y_dd.push_back(qp.calc_second_derivative(tp));
            fp1.y_ddd.push_back(qp.calc_third_derivative(tp));
            fp1.c_deviation += dt_t*sq(qp.calc_point(tp));
            // fp1.c_velocity += dt_t*sq(qp.calc_first_derivative(tp));
            // fp1.c_accel += dt_t*sq(qp.calc_second_derivative(tp));
            // if(qp.calc_point(tp) * qp.calc_first_derivative(tp) > 1e-6){
            //     fp1.c_deviation += dt_t*sq(27.0/256.0*pow(abs(qp.calc_first_derivative(tp)*t.back() +4.0*qp.calc_point(tp)), 4.0)/pow(abs(qp.calc_first_derivative(tp)*t.back() +3.0*qp.calc_point(tp)), 3.0));    
            // }else{
            //     fp1.c_deviation += dt_t*sq(qp.calc_point(tp));
            // }
            fp1.c_velocity = max(fp1.c_velocity,sq(qp.calc_first_derivative(tp)));
            fp1.c_accel = max(fp1.c_accel,sq(qp.calc_second_derivative(tp)));
            fp1.c_jerk += dt_t*sq(qp.calc_third_derivative(tp)); 
        }
        fp1.c_end = sq( fp1.y.back() - target_y );
        fp1s.push_back(fp1);
    }
    return fp1s;
}