#include "cubic_spline_2d.h"
#include "utils.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

using namespace std;

CubicSpline2D::CubicSpline2D(const vector<double> &x,
                             const vector<double> &y,
                             const vector<double> &z) {
    /*vector<vector<double>> filtered_points = remove_collinear_points(x, y);
    calc_s(filtered_points[0], filtered_points[1]);
    this->x = filtered_points[0];
    this->y = filtered_points[1];
    sx = CubicSpline1D(s, filtered_points[0]);
    sy = CubicSpline1D(s, filtered_points[1]);
    */
   calc_s(x,y);
   this->x = x;
   this->y = y;
   this->z = z;
   sx = CubicSpline1D(s,x);
   sy = CubicSpline1D(s,y);
   sz = CubicSpline1D(s,z);
   size = s.size();
   length = s[size-1];
}


CubicSpline2D::CubicSpline2D(const vector<double> &x, const vector<double> &y, const vector<double> &z, const vector<double> &s,
                      const vector<vector<double>> &coff_x, const vector<vector<double>> &coff_y, const vector<vector<double>> &coff_z, const vector<int> &edg,
                      vector<string> id, vector<double> boundary, int size, int index, double dist){
    this->x = x;
    this->y = y;
    this->z = z;
    this->s = s;
    sx = CubicSpline1D(s,x,coff_x);
    sy = CubicSpline1D(s,y,coff_y);
    sz = CubicSpline1D(s,y,coff_z);
    this->id = id;
    this->boundary = boundary;
    this->size = size;
    this->index = index;
    this->edge = edg;
    length = s[size-1];
    this->dist = dist;
}

        

void CubicSpline2D::calc_s(const vector<double>& x,
                           const vector<double>& y) {
    int nx = x.size();
    vector<double> dx (nx);
    vector<double> dy (nx);
    adjacent_difference(x.begin(), x.end(), dx.begin());
    adjacent_difference(y.begin(), y.end(), dy.begin());
    dx.erase(dx.begin());
    dy.erase(dy.begin());

    double cum_sum = 0.0;
    s.push_back(cum_sum);
    for (int i = 0; i < nx - 1; i++) {
        cum_sum += norm(dx[i], dy[i]);
        s.push_back(cum_sum);
    }
    // s.erase(unique(s.begin(), s.end()), s.end());
}

double CubicSpline2D::calc_x(double t) {
    return sx.calc_der0(t);
}

double CubicSpline2D::calc_y(double t) {
    return sy.calc_der0(t);
}

double CubicSpline2D::calc_z(double t) {
    return sz.calc_der0(t);
}

double CubicSpline2D::calc_curvature(double t){
    double dx = sx.calc_der1(t);
    double ddx = sx.calc_der2(t);
    double dy = sy.calc_der1(t);
    double ddy = sy.calc_der2(t);
    double k;
    if( hypot(dx,dy) < 1e-6 ){
        k = 0;
    } else{
        k = (ddy * dx - ddx * dy) /
            pow(pow(dx, 2) + pow(dy, 2), 1.5);
    }
    return k;
}

double CubicSpline2D::calc_yaw(double t) {
    double dx = sx.calc_der1(t);
    double dy = sy.calc_der1(t);
    double yaw;
    if(hypot(dx,dy)<1e-6){
        double ddx = sx.calc_der2(t);
        double ddy = sy.calc_der2(t);
        if(hypot(ddx,ddy)<1e-6){
            yaw = 0;
            ROS_ERROR("hard stopped spline");
        }
        else{
            yaw = atan2(ddy,ddx);
            ROS_ERROR("weak stopped spline");
        }
    }
    else{
        yaw = atan2(dy, dx);
    }
    return yaw;
}

double CubicSpline2D::calc_acoserr(double t){
    double dx = sx.calc_der1(t);
    double dy = sy.calc_der1(t);
    return hypot(dx,dy);
}

double CubicSpline2D::calc_ratiorate(double t, double l){
    double dx = sx.calc_der1(t);
    double dy = sy.calc_der1(t);
    double ddx = sx.calc_der2(t);
    double ddy = sy.calc_der2(t);
    double dddx = sx.calc_der3(t);
    double dddy = sy.calc_der3(t);
    double acoserr = calc_acoserr(t);
    double cplus = dx*ddx + dy*ddy;
    double cminus = dx*ddy - dy*ddx;
    double cc = dx*dddy - dy*dddx;
    return cplus/acoserr + (-cc + 2*cplus*cminus/sq(acoserr)) /sq(acoserr) *l;
}

double CubicSpline2D::find_s(double x, double y, double s0, bool accurate, double step) {    //accurate=true, step=0.1 used only when accurate == false
    // 가장 가까운 i-1 to i 선분을 찾음 
    int i_closest = 1;
    double closest = INFINITY;
    int ssize = s.size();
    double s1 = s[0]; 
    double x1 = calc_x(s1);
    double y1 = calc_y(s1);
    int i = 1;
    while(i<ssize){
        double s2 = s[i];
        double x2 = calc_x(s2);
        double y2 = calc_y(s2);
        double dist = distance_to_segment(x,y,x1,y1,x2,y2);
        if (dist < closest) {
            closest = dist;
            i_closest = i;
        }
        i+=1;
        s1 = s2;
        x1 = x2;
        y1 = y2;
    }
    if((!accurate)&&(closest>3.0)) return s[i_closest];
    // i-2 to i+1 세 선분만 탐색
    double s_closest = s[i_closest];
    closest = INFINITY;
    double si = s[max(0,i_closest-2)];
    do {
        double px = calc_x(si);
        double py = calc_y(si);
        double dist = norm(x - px, y - py);
        if (dist < closest) {
            closest = dist;
            s_closest = si;
        }
        if(accurate==true){
            si += 0.005;
        }else{
            si += step;
        }
    } while (si < s[min(ssize-1,i_closest+1)]);

    //범위 판정
    if(accurate){
        if(s_closest<s.front()+0.1){
            double px = calc_x(s.front());
            double py = calc_y(s.front());
            double pyaw = calc_yaw(s.front());
            if( (x-px)*cos(pyaw) + (y-py)*sin(pyaw) < -0.005){
                //do something
                return s.front() + (x-px)*cos(pyaw) + (y-py)*sin(pyaw);
            }
        }else if(s_closest>s.back()-0.1){
            double px = calc_x(s.back());
            double py = calc_y(s.back());
            double pyaw = calc_yaw(s.back());
            if( (x-px)*cos(pyaw) + (y-py)*sin(pyaw) > 0.005){
                //do something
                return s.back() + (x-px)*cos(pyaw) + (y-py)*sin(pyaw);
            }
        }
    }
    return s_closest;
}

SLState CubicSpline2D::transform(PoseState ps, bool accurate){  //accurate=true
    SLState sls;
    sls.s = find_s(ps.x,ps.y,s.front(),accurate);
    if(sls.s<s.front()){
        //do something
    }else if(sls.s>s.back()){
        //do something
    }
    double px = calc_x(sls.s);
    double py = calc_y(sls.s);
    double pyaw = calc_yaw(sls.s);
    double pcurvature = calc_curvature(sls.s);
    double pacoserr = 1;
    // int i = std::upper_bound (s.begin(), s.end(), sls.s) - s.begin();
    //if(s.back()==sls.s) i=s.size()-1;
    // double erryaw;
    // if( hypot(calc_x(s[i])-calc_x(s[i-1]),calc_y(s[i])-calc_y(s[i-1]) < 1e-6 )) erryaw = 0;
    // else erryaw = pyaw - atan2( calc_y(s[i])-calc_y(s[i-1]), calc_x(s[i])-calc_x(s[i-1]) );
    if(1-pcurvature*sls.l<1e-1) ROS_ERROR("[CSP] too big l %lf/%lf",pcurvature,sls.l);
    sls.l = (ps.x-px)*(-sin(pyaw)) + (ps.y-py)*cos(pyaw);
    if(accurate==true){
        pacoserr = calc_acoserr(sls.s);
        sls.s += ((ps.x-px)*cos(pyaw) + (ps.y-py)*sin(pyaw))/(1-pcurvature*sls.l)/pacoserr;
        px = calc_x(sls.s);
        py = calc_y(sls.s);
        pyaw = calc_yaw(sls.s);
        pcurvature = calc_curvature(sls.s);
        if(1-pcurvature*sls.l<1e-1) ROS_ERROR("[CSP] too big l %lf/%lf",pcurvature,sls.l);
    }else return sls;
    pacoserr = calc_acoserr(sls.s);
    double pratiorate = calc_ratiorate(sls.s,sls.l);
    if(pacoserr<0.9) ROS_ERROR("[CSP] too small acoserr %lf %s %s %s",pacoserr, id[0].c_str(), id[1].c_str(), id[2].c_str());
    Pose p = ps.getPose();
    p[2] = pyaw;
    PoseState transformed = ps.transform(p);
    sls.ds = transformed.vx/(1-pcurvature*sls.l)/pacoserr;
    sls.dl = transformed.vy;
    sls.dds = (transformed.ax - pratiorate*sq(sls.ds) + 2*pacoserr*pcurvature*sls.dl*sls.ds) /pacoserr /(1-pcurvature*sls.l); 
    sls.ddl = transformed.ay - sq(pacoserr)*pcurvature*(1-pcurvature*sls.l)*sq(sls.ds);
    return sls;
}

PoseState CubicSpline2D::sl_to_xy(SLState sls){
    PoseState ps;
    double ix_, iy_, iyaw_, icurvature_, iacoserr_, iratiorate_, di;
    ix_ = calc_x(sls.s);
    iy_ = calc_y(sls.s);
    //if (isnan(ix_) || isnan(iy_)) ROS_ERROR("csp : nan xy");
    iyaw_ = calc_yaw(sls.s);
    // int i = std::upper_bound (s.begin(), s.end(), sls.s) - s.begin();
    // x[0] 미만이면 0, [x[i-1],x[i]) 이면 i, x.back() 이상이면 (int)(x.size())
    //if(s.back()==sls.s) i=s.size()-1;
    // double erryaw;
    // if( hypot(calc_x(s[i])-calc_x(s[i-1]),calc_y(s[i])-calc_y(s[i-1]) < 1e-6 )) erryaw = 0;
    // else erryaw = iyaw_ - atan2( calc_y(s[i])-calc_y(s[i-1]), calc_x(s[i])-calc_x(s[i-1]) );
    // if(cos(erryaw)<0.9) ROS_ERROR("too big erryaw");
    
    icurvature_ = calc_curvature(sls.s);
    iacoserr_= calc_acoserr(sls.s);
    iratiorate_ = calc_ratiorate(sls.s,sls.l);
    ps.x = 0;
    ps.y = sls.l;
    ps.vx = iacoserr_ * (1-icurvature_*sls.l) *sls.ds;
    ps.vy = sls.dl;
    ps.ax = iacoserr_*(1-icurvature_*sls.l)*sls.dds + iratiorate_*sq(sls.ds) - 2*iacoserr_*icurvature_*sls.dl*sls.ds; 
    ps.ay = sls.ddl + sq(iacoserr_)*icurvature_*(1-icurvature_*sls.l)*sq(sls.ds);

    Pose pose;
    pose.assign({ix_,iy_,iyaw_});
    return ps.transform(pose,true);
}


vector<vector<double>>
CubicSpline2D::remove_collinear_points(vector<double> x, vector<double> y) {
    vector<vector<double>> filtered_points;
    vector<double> x_, y_;
    x_.push_back(x[0]);
    x_.push_back(x[1]);
    y_.push_back(y[0]);
    y_.push_back(y[1]);
    for (size_t i = 2; i < x.size()-1; i++) {
        bool collinear = are_collinear(
            x[i - 2], y[i - 2],
            x[i - 1], y[i - 1],
            x[i], y[i]
            );
        if (collinear) {
            continue;
        }
        x_.push_back(x[i]);
        y_.push_back(y[i]);
    }
    
    x_.push_back(x.back());
    y_.push_back(y.back());
    filtered_points.push_back(x_);
    filtered_points.push_back(y_);
    return filtered_points;
}

bool CubicSpline2D::are_collinear(double x1, double y1, double x2, double y2,
    double x3, double y3) {
    double a = x1 * (y2 - y3) +
               x2 * (y3 - y1) +
               x3 * (y1 - y2);
    return a <= 0.01;
}

double CubicSpline2D::calc_s_length(){
    return s.back();
}


void CubicSpline2D::set_index(int index){
    this->index = index;
}

int CubicSpline2D::get_index(){
    return index;
}

vector<string> CubicSpline2D::get_id(){
    return id;
}

int CubicSpline2D::get_size(){
    return size;
}

vector<double> CubicSpline2D::get_x(){
    return x;
}

vector<double> CubicSpline2D::get_y(){
    return y;
}

vector<double> CubicSpline2D::get_z(){
    return z;
}

vector<double> CubicSpline2D::get_s(){
    return s;
}

CubicSpline1D CubicSpline2D::get_sx(){
    return sx;
}

CubicSpline1D CubicSpline2D::get_sy(){
    return sy;
}

CubicSpline1D CubicSpline2D::get_sz(){
    return sz;
}

double CubicSpline2D::get_length(){
    return length;
}

vector<int> CubicSpline2D::get_edge(){
    return edge;
}

vector<double> CubicSpline2D::get_boundary(){
    return boundary;
}

void CubicSpline2D::set_id(vector<string> id){
    this->id = id;
}

void CubicSpline2D::set_boundary(vector<double> boundary){
    this->boundary = boundary;
}

void CubicSpline2D::set_dist(double dist){
    this->dist = dist;
}

double CubicSpline2D::get_dist(){
    return dist;
}
