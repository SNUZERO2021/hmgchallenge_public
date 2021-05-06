#ifndef FRENET_OPTIMAL_TRAJECTORY_FRENET_PATH_H
#define FRENET_OPTIMAL_TRAJECTORY_FRENET_PATH_H

#include "cpp_struct.h"
#include "cubic_spline_2d.h"
#include "car.h"
#include "obstacle.h"
#include <Eigen/Dense>
#include <vector>
#include <tuple>

using namespace std;
using namespace Eigen;

class FrenetPath {
 public:
    int flag;
    
    int cspno;
    // Frenet attributes
    vector<double> t;          // time
    vector<double> d;          // lateral offset
    vector<double> d_d;        // lateral speed
    vector<double> d_dd;       // lateral acceleration
    vector<double> d_ddd;      // lateral jerk
    vector<double> s;          // s position along spline
    vector<double> s_d;        // s speed
    vector<double> s_dd;       // s acceleration
    vector<double> s_ddd;      // s jerk

    // Euclidean attributes
    vector<double> x;          // x position
    vector<double> y;          // y position
    vector<double> yaw;        // yaw in rad
    vector<double> ds;         // speed
    vector<double> c;          // curvature
    vector<double> accel;      // acceleration

    // Debug
    vector<double> ix;
    vector<double> iy;
    vector<double> iyaw;

    // Cost attributes
    // lateral costs
    double c_lateral_deviation = 0.0;
    double c_lateral_velocity = 0.0;
    double c_lateral_acceleration = 0.0;
    double c_lateral_jerk = 0.0;
    double c_lateral_end = 0.0;
    double c_lateral = 0.0;

    // longitudinal costs
    double c_longitudinal_acceleration = 0.0;
    double c_longitudinal_jerk = 0.0;
    double c_longitudinal_end = 0.0;
    double c_longitudinal = 0.0;

    // time costs
    double c_time_taken = 0.0;

    // obstacle costs
    double c_inv_dist_to_obstacles = 0.0;

    // decision costs
    double c_decision = 0.0;

    // final cost
    double cf = 0.0;

    static int fail_speed_count;
    static int fail_accel_count;
    static int fail_break_count;
    static int fail_curvature_count;
    static int fail_obstacle_count;

    static double time0;
    static double time1;
    static double time2;
    static double time3;
    static double time4;
    static double time5;

    FrenetPath();
    FrenetPath(FrenetHyperparameters *fot_hp_);
    FrenetPath& operator=(const FrenetPath& o);
    bool to_global_path(CubicSpline2D* csp);
    bool is_valid_path(const vector<vector<ConvexHull>> &dynamic_obstacles, const vector<ConvexHull> &static_obstacles);
    bool is_collision(const vector<vector<ConvexHull>> &dynamic_obstacles, const vector<ConvexHull> &static_obstacles);
    double inverse_distance_to_obstacles(const vector<vector<ConvexHull>> &dynamic_obstacles, const vector<ConvexHull> &static_obstacles);

 private:
    // Hyperparameters
    FrenetHyperparameters *fot_hp;
};

#endif
