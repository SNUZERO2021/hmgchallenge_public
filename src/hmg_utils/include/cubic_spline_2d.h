#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBIC_SPLINE_2D
#define FRENET_OPTIMAL_TRAJECTORY_CUBIC_SPLINE_2D

#include "cubic_spline_1d.h"
#include "ros/ros.h"
#include "utils.h"
#include <vector>

using namespace std;

enum Edge{
    NEXT_STRAIGHT,
    NEXT_LEFT,
    NEXT_RIGHT,
    LEFT_STRAIGHT,
    LEFT_LEFT,
    LEFT_RIGHT,
    RIGHT_STRAIGHT,
    RIGHT_LEFT,
    RIGHT_RIGHT,
    CURRENT_IS_INTER,
    NEXT_IS_INTER,
    CURRENT_SECTOR_NUM,
    NEXT_SECTOR_NUM,
    LANE_NUM1,
    LANE_NUM2,
};

class CubicSpline2D{
    private:
        vector<string> id;
        int index;
        vector<int> edge;
        vector<double> boundary;
        int size;
        double length;
        double dist;
        vector<double> x,y,z,s;
        CubicSpline1D sx, sy, sz;
        void calc_s(const vector<double>& x, const vector<double>& y);
        vector<vector<double>> remove_collinear_points(vector<double> x, vector<double> y);
        bool are_collinear(double x1, double y1, double x2, double y2, double x3, double y3);
    public:
        CubicSpline2D() = default;
        CubicSpline2D(const vector<double> &x, const vector<double> &y, const vector<double> &z);
        CubicSpline2D(const vector<double> &x, const vector<double> &y, const vector<double> &z, const vector<double> &s,
                      const vector<vector<double>> &coff_x, const vector<vector<double>> &coff_y, const vector<vector<double>> &coff_z, const vector<int> &edg,
                      vector<string> id, vector<double> boundary, int size, int index, double dist);
        
        double calc_x(double t);
        double calc_y(double t);
        double calc_z(double t);
        double calc_curvature(double t);
        double calc_yaw(double t);
        double calc_acoserr(double t);
        double calc_ratiorate(double t, double l);
        double find_s(double x, double y, double s0, bool accurate=true, double step=0.1);
        SLState transform(PoseState ps, bool accurate=true);
        PoseState sl_to_xy(SLState sls);
        double calc_s_length();
        void set_id(vector<string> id);
        void set_boundary(vector<double> boundary);
        void set_index(int index);
        void set_dist(double dist);
        double get_dist();
        int get_index();
        vector<string> get_id();
        int get_size();
        vector<double> get_x();
        vector<double> get_y();
        vector<double> get_z();
        vector<double> get_s();
        CubicSpline1D get_sx();
        CubicSpline1D get_sy();
        CubicSpline1D get_sz();
        double get_length();
        vector<double> get_boundary();
        vector<int> get_edge();
};

#endif