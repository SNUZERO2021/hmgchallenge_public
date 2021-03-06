#ifndef FRENET_OPTIMAL_TRAJECTORY_CUBIC_SPLINE_1D
#define FRENET_OPTIMAL_TRAJECTORY_CUBIC_SPLINE_1D

#include <Eigen/LU>
#include <vector>

using namespace std;
using namespace Eigen;

class CubicSpline1D{
    private:
        vector<double> a, b, c, d, w, x, y;
        int search_index(double t);
        void matrix_a(vector<double>& deltas, MatrixXd& result);
        void vector_b(vector<double>& deltas, VectorXd& result);
    public:
        int nx;
        CubicSpline1D() = default;
        CubicSpline1D(const vector<double>& v1, const vector<double>& v2);
        CubicSpline1D(const vector<double> &x, const vector<double>& y, const vector<vector<double>> &coff);
        double calc_der0(double t);
        double calc_der1(double t);
        double calc_der2(double t);
        double calc_der3(double t);
        vector<double> get_x();
        vector<double> get_y();
        vector<vector<double>> get_coefficient();
};

#endif