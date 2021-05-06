#include "frenet_path.h"
#include "utils.h"
#include "ros/console.h"

#include <chrono>
#include <math.h>
#include <algorithm>

const double static_obstacle_clearance = 0.1;
const double dynamic_obstacle_clearance = 0.1;

const float COLLISION_CHECK_THRESHOLD = 0.1;
int FrenetPath::fail_speed_count = 0;
int FrenetPath::fail_accel_count = 0;
int FrenetPath::fail_break_count = 0;
int FrenetPath::fail_curvature_count = 0;    
int FrenetPath::fail_obstacle_count = 0;

// double FrenetPath::time0 = 0.0;
// double FrenetPath::time1 = 0.0;
// double FrenetPath::time2 = 0.0;
// double FrenetPath::time3 = 0.0;
// double FrenetPath::time4 = 0.0;
// double FrenetPath::time5 = 0.0;

FrenetPath::FrenetPath(){
    flag = -1;
}

FrenetPath::FrenetPath(FrenetHyperparameters *fot_hp_) {
    fot_hp = fot_hp_;
}

FrenetPath& FrenetPath::operator=(const FrenetPath& o){

    fot_hp = o.fot_hp;

    flag = o.flag;
    
    cspno = o.cspno;
    
    t = o.t;          // time
    d = o.d;          // lateral offset
    d_d = o.d_d;        // lateral speed
    d_dd = o.d_dd;       // lateral acceleration
    d_ddd = o.d_ddd;      // lateral jerk
    s = o.s;          // s position along spline
    s_d = o.s_d;        // s speed
    s_dd = o.s_dd;       // s acceleration
    s_ddd = o.s_ddd;      // s jerk

    x = o.x;          // x position
    y = o.y;          // y position
    yaw = o.yaw;        // yaw in rad
    ds = o.ds;         // speed
    c = o.c;          // curvature
    accel = o.accel;      // acceleration

    // Debug
    ix = o.ix;
    iy = o.iy;
    iyaw = o.iyaw;

    // Cost attributes
    // lateral costs
    c_lateral_deviation = o.c_lateral_deviation;
    c_lateral_velocity = o.c_lateral_velocity;
    c_lateral_acceleration = o.c_lateral_acceleration;
    c_lateral_jerk = o.c_lateral_jerk;
    c_lateral_end = o.c_lateral_end;
    c_lateral = o.c_lateral;

    // longitudinal costs
    c_longitudinal_acceleration = o.c_longitudinal_acceleration;
    c_longitudinal_jerk = o.c_longitudinal_jerk;
    c_longitudinal_end = o.c_longitudinal_end;
    c_longitudinal = o.c_longitudinal;

    // time costs
    c_time_taken = o.c_time_taken;

    // obstacle costs
    c_inv_dist_to_obstacles = o.c_inv_dist_to_obstacles;

    // decision costs
    c_decision = o.c_decision;

    // final cost
    cf = o.cf;
}

bool FrenetPath::to_global_path(CubicSpline2D* csp) {
    double ix_, iy_, iyaw_, icurvature_, di, fx, fy, dx, dy, ddx, ddy;
    // calc global positions
    for (size_t i = 0; i < s.size(); i++) {
        SLState sls;
        sls.s=s[i];
        sls.l=d[i];
        sls.ds=s_d[i];
        sls.dl=d_d[i]*s_d[i];
        sls.dds=s_dd[i];
        sls.ddl=d_dd[i]*s_d[i]*s_d[i] + d_d[i]*s_dd[i];
        PoseState ps = csp->sl_to_xy(sls);

        // ix_ = csp->calc_x(s[i]);
        // iy_ = csp->calc_y(s[i]);
        // if (isnan(ix_) || isnan(iy_)) break;
        // iyaw_ = csp->calc_yaw(s[i]);
        // ix.push_back(ix_);
        // iy.push_back(iy_);
        // iyaw.push_back(iyaw_);
        
        // printf("s %lf l %lf x %lf y %lf i %d s.size %d l.size %d \n",s[i],d[i],ps.x,ps.y,i,s.size(),d.size());
        // cout<<endl;

        if (isnan(ps.x) || isnan(ps.y)) break;
        x.push_back(ps.x);
        y.push_back(ps.y);
        ds.push_back(hypot(ps.vx,ps.vy));
        if(hypot(ps.vx,ps.vy)<1e-1){
        //if(true){
            yaw.push_back(csp->calc_yaw(sls.s));
            c.push_back(csp->calc_curvature(sls.s));
            //accel.push_back(hypot(ps.ax,ps.ay));
            accel.push_back( ps.ax*cos(csp->calc_yaw(sls.s)) + ps.ay*sin(csp->calc_yaw(sls.s)) );
        }
        else{
            yaw.push_back(atan2(ps.vy,ps.vx));
            c.push_back( (ps.vx*ps.ay - ps.ax*ps.vy) / (hypot(ps.vx,ps.vy)*hypot(ps.vx,ps.vy)*hypot(ps.vx,ps.vy)) );
            accel.push_back( (ps.vx*ps.ax+ps.vy*ps.ay)/hypot(ps.vx,ps.vy) );
        }
        //if(isnan(c.back())||isnan(accel.back())) ROS_ERROR("nan c accel, %d, %d, %d, %d",isnan(c.back()),isnan(accel.back()),isnan(ps.ax),isnan(ps.ay));
    }

    if (x.size() <= 3) {    ///
        return false;
    }

    // for (size_t i = 0; i < x.size() - 1; i++) {
    //     dx = x[i+1] - x[i];
    //     dy = y[i+1] - y[i];
    //     yaw.push_back(atan2(dy, dx));
    //     ds.push_back(hypot(dx, dy));
    // }
    // yaw.push_back(yaw.back());
    // ds.push_back(ds.back());

    // for (size_t i = 0; i < yaw.size() - 1; i++) {
    //     double dyaw = yaw[i+1] - yaw[i];
    //     if (dyaw > M_PI_2) {
    //         dyaw -= M_PI;
    //     } else if (dyaw < -M_PI_2) {
    //         dyaw += M_PI;
    //     }
    //     c.push_back(dyaw / ds[i]);
    // }

    return true;
}

bool FrenetPath::is_valid_path(
        const vector<vector<ConvexHull>> &dynamic_obstacles, 
        const vector<ConvexHull> &static_obstacles) {
    int skip = (int)(t.size())/6;
    //skip=2;
    if (any_of(ds.begin()+skip, ds.end(),
            [this](double i){return abs(i) > fot_hp->max_speed;})) {
        fail_speed_count++;
        // for(int i=0; i<ds.size(); i++){
        //     ROS_ERROR("i %d s%.3lf s_d %.3lf s_dd %.3lf d_d %.3lf d_dd %.3lf ds %.3lf t %.3lf tsize %d",i,s[i], s_d[i], s_dd[i], d_d[i], d_dd[i], ds[i], t[i], (int)t.size());
        // }
        return false;
    }

    if (any_of(s_d.begin(), s_d.begin()+1,
            [this](double i){return abs(i) < 1e-6;})) {
        fail_speed_count++;
        ROS_ERROR("[PP:FP]negative path");
        return false;
    }
    
    else if (any_of(accel.begin()+skip, accel.end(),
            [this](double i){return i > fot_hp->max_accel;})) {
        fail_accel_count++;
        return false;
    }

    else if (any_of(accel.begin()+skip, accel.end(),
            [this](double i){return i < -fot_hp->max_break;})) {
        fail_break_count++;
        return false;
    }
    
    else if (any_of(c.begin()+skip, c.end(),
            [this](double i){return abs(i) > fot_hp->max_curvature;})) {
        fail_curvature_count++;
        return false;
    }
    //if(false)return true;
    else if (is_collision(dynamic_obstacles,static_obstacles)) {
        fail_obstacle_count++;
        return false;
    }
    else {
        return true;
    }
}

bool FrenetPath::is_collision(
        const vector<vector<ConvexHull>> &dynamic_obstacles, 
        const vector<ConvexHull> &static_obstacles) {
    Pose pose;
    Car car = Car();
    vector<point> car_corners;
    vector<point> prev_car_corners;

    double min_dist = INF;

    int i = 0;
    for (i = 0; i < min<int>(x.size()-1,dynamic_obstacles.size()); i++) {
        pose.assign({x[i], y[i], yaw[i]});
        car.setPose(pose);
        prev_car_corners = car.getCorner();
        pose.assign({x[i+1], y[i+1], yaw[i+1]});
        car.setPose(pose);
        car_corners = car.getCorner();
        prev_car_corners.insert(prev_car_corners.end(), car_corners.begin(), car_corners.end());
        ConvexHull car_hull = ConvexHull(prev_car_corners);
        for(auto & obstacle_hull : dynamic_obstacles[i]){
            double tmp = gjk(car_hull.p, obstacle_hull.p);
            if(tmp < dynamic_obstacle_clearance){
                return true;
            }
            min_dist = min(min_dist, tmp);
        }
        for(auto & obstacle_hull : static_obstacles){
            double tmp = gjk(car_hull.p, obstacle_hull.p);
            if(tmp < static_obstacle_clearance){
                return true;
            }
            min_dist = min(min_dist, tmp);
        }
    }
    /**** TODO : Look Ahead Check ****/
    // pose.assign({x[i], y[i], yaw[i]});
    // car.setPose(pose);
    // prev_car_corners = car.getCorner();
    // pose.assign({x[i]+1.0*cos(yaw[i]), y[i]+1.0*sin(yaw[i]), yaw[i]});
    // car.setPose(pose);
    // car_corners = car.getCorner();
    // prev_car_corners.insert(prev_car_corners.end(), car_corners.begin(), car_corners.end());
    // ConvexHull car_hull = ConvexHull(prev_car_corners);
    // for(auto & obstacle_hull : static_obstacles){
    //     double tmp = gjk(car_hull.p, obstacle_hull.p);
    //     if(tmp < EPS){
    //         return true;
    //     }
    //     min_dist = min(min_dist, tmp);
    // }

    c_inv_dist_to_obstacles = sq(1/max(min_dist,EPS));
    return false;
}