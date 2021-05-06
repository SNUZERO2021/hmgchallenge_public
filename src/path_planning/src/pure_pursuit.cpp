#include "pure_pursuit.h"
#include <math.h>

using namespace std;

typedef pair<double, double> pdd;

const double EPS = 1e-2;

int ccw(pdd a, pdd b){
    //a:ref, b:target
    double val = a.first*b.second-a.second*b.first;
    if(abs(val)<EPS) return 0;
    if(val<0) return -1;
    return 1;
}

PurePursuit::PurePursuit(){
}

PurePursuit::PurePursuit(double theta, double xs1, double ys1, double xf1, double yf1, vector<double> ss){
    //curvature;
    xs = xs1;
    ys = ys1;
    xf = xf1;
    yf = yf1;
    s = ss;
    u = cos(theta);
    v = sin(theta);
    pdd target = {xf-xs, yf-ys};
    double a = target.first;
    double b = target .second;
    int dir = ccw({u,v},target);
    if(dir == 0){
        // straight path
        curvature = 0.0;
        for(double l : s){
            x.push_back(xs+u*l);
            y.push_back(ys+v*l);
        }
    }
    else{
        double xr = 0.5*(a*a+b*b)/(a*v-b*u)*v;
        double yr = -0.5*(a*a+b*b)/(a*v-b*u)*u;
        curvature = dir*1.0/abs(0.5*(a*a+b*b)/(a*v-b*u));
        double theta0 = atan2(-yr, -xr);
        for(double l : s){
            x.push_back(xs + xr + cos(theta0+l*curvature)/abs(curvature));
            y.push_back(ys + yr + sin(theta0+l*curvature)/abs(curvature));
        }
    }

}
