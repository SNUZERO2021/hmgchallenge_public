#include "cubic_spline_1d.h"

#include <algorithm>
#include <numeric>
#include <cmath>

using namespace std;
using namespace Eigen;

CubicSpline1D::CubicSpline1D(const vector<double>& v1, const vector<double>& v2):nx(v1.size()), a(v2), x(v1), y(v2){
    
    vector<double> deltas (nx);
    adjacent_difference(x.begin(), x.end(), deltas.begin());
    deltas.erase(deltas.begin());

    MatrixXd ma = MatrixXd::Zero(nx, nx);
    VectorXd vb = VectorXd::Zero(nx);
    matrix_a(deltas, ma);
    vector_b(deltas, vb);

    MatrixXd ma_inv = ma.inverse();
    VectorXd tmp_c = ma_inv * vb;
    c.resize(tmp_c.size());
    VectorXd::Map(&c[0], tmp_c.size()) = tmp_c;

    for (int i = 0; i < nx - 1; i++) {
        d.push_back((c[i + 1] - c[i]) / (3.0 * deltas[i]));
        b.push_back((a[i + 1] - a[i]) / deltas[i] - deltas[i] * (c[i + 1] + 2.0 * c[i]) / 3.0);
    }
}

CubicSpline1D::CubicSpline1D(const vector<double> &x, const vector<double>& y, const vector<vector<double>> &coff){
    this->x = x;
    this->y = y;
    a = coff[0];
    b = coff[1];
    c = coff[2];
    d = coff[3];
}

double CubicSpline1D::calc_der0(double t) {
    // if (t < x.front() || t > x.back()) {
    //     return NAN;
    // }

    int i = search_index(t) - 1;
    // x[0] 미만이면 -1, [x[i],x[i+1]) 이면 i, x.back() 이상이면 (int)(x.size())-1
    if(i<0) i=0;
    if(i>=(int)(x.size())-1) i = (int)(x.size())-2;
    double dx = t - x[i];
    
    return a[i] + b[i] * dx + c[i] * pow(dx, 2) + d[i] * pow(dx, 3);
}

double CubicSpline1D::calc_der1(double t) {
    // if (t < x.front() || t > x.back()) {
    //     return NAN;
    // }

    int i = search_index(t) - 1;
    // x[0] 미만이면 -1, [x[i],x[i+1]) 이면 i, x.back() 이상이면 (int)(x.size())-1
    if(i<0) i=0;
    if(i>=(int)(x.size())-1) i = (int)(x.size())-2;
    double dx = t - x[i];

    return b[i] + 2.0 * c[i] * dx + 3.0 * d[i] * pow(dx, 2);
}

double CubicSpline1D::calc_der2(double t) {
    // if (t < x.front() || t > x.back()) {
    //     return NAN;
    // }

    int i = search_index(t) - 1;
    // x[0] 미만이면 -1, [x[i],x[i+1]) 이면 i, x.back() 이상이면 (int)(x.size())-1
    if(i<0) i=0;
    if(i>=(int)(x.size())-1) i = (int)(x.size())-2;
    double dx = t - x[i];

    return 2.0 * c[i] + 6.0 * d[i] * dx;
}

double CubicSpline1D::calc_der3(double t) {
    // if (t < x.front() || t > x.back()) {
    //     return NAN;
    // }

    int i = search_index(t) - 1;
    // x[0] 미만이면 -1, [x[i],x[i+1]) 이면 i, x.back() 이상이면 (int)(x.size())-1
    if(i<0) i=0;
    if(i>=(int)(x.size())-1) i = (int)(x.size())-2;
    double dx = t - x[i];

    return 6.0 * d[i];
}

void CubicSpline1D::matrix_a(vector<double> &deltas, MatrixXd &result) {
    result(0, 0) = 1.0;
    for (int i = 0; i < nx - 1; i++) {
        if (i != nx - 2) {
            result(i + 1, i + 1) = 2.0 * (deltas[i] + deltas[i + 1]);
        }
        result(i + 1, i) = deltas[i];
        result(i, i + 1) = deltas[i];
    }

    result(0, 1) = -1.0;    // 0.0 for double derivative boundary condition
    result(nx - 1, nx - 2) = -1.0;  //0.0 for double derivative boundary condition
    result(nx - 1, nx - 1) = 1.0;
}

void CubicSpline1D::vector_b(vector<double> &deltas, VectorXd &result) {
    for (int i = 0; i < nx - 2; i++) {
        result(i + 1) = 3.0 * (a[i + 2] - a[i + 1]) / deltas[i + 1] - 3.0 * (a[i + 1] - a[i]) / deltas[i];
    }
}

int CubicSpline1D::search_index(double t) {
    //if(x.back()==t) return x.size()-1;
    return std::upper_bound (x.begin(), x.end(), t) - x.begin();
    // x[0] 미만이면 0, [x[i-1],x[i]) 이면 i, x.back() 이상이면 (int)(x.size())
}

vector<double> CubicSpline1D::get_x(){
    return x;
}

vector<double> CubicSpline1D::get_y(){
    return y;
}

vector<vector<double>> CubicSpline1D::get_coefficient(){
    vector<vector<double>> rt;
    
    rt.push_back(a);
    rt.push_back(b);
    rt.push_back(c);
    rt.push_back(d);

    return rt;
}