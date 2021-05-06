#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include <iostream>
#include <vector>


using namespace std;

typedef pair<double, double> pdd;

class PurePursuit{
    public:
    double curvature;
    double xs,ys;
    double xf,yf;
    double u,v;
    vector<double> x;
    vector<double> y;
    vector<double> s;


    PurePursuit();
    PurePursuit(double theta, double xs1, double ys1, double xf1, double yf1, vector<double> ss);
    


};

#endif