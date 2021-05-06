#ifndef PROJ_H
#define PROJ_H

#include <cmath>
#include <tuple>
#include <vector>
#include <stack>
#include <ros/console.h>
#include "ros/ros.h"

vector<double> latlong_to_xy(double lati, double longi){
    lati = (lati-37.58) * 10000;
    longi = (longi - 126.89) * 10000;
    double a = -1.0526565e-01*lati +  8.8334789e+00*longi +  3.4405369e-01;
    double b = 1.1098592e+01*lati +  8.3344474e-02*longi +  1.4646138e-01;
    double c = 4.8751599e-07*lati +  4.2407201e-08*longi +  1.0000000e+00;
    // ROS_INFO("a b c a/c b/c %lf %lf %lf %lf %lf",a,b,c,a/c,b/c);
    return vector<double>({a/c, b/c});
}
vector<double> xy_to_latlong(double x, double y){
    // ROS_ERROR("xy_to_latlong NOT DEFINED YET");
    return vector<double>({-1.0,-1.0});
}

#endif