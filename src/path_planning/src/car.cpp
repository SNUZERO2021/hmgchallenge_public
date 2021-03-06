#include "car.h"
#include <cmath>
#include "ros/console.h"

void Car::setPose(Pose p){
    pose = p;
    poseState.setPose(pose);
}
void Car::setPose(PoseState ps){
    poseState=ps;
    pose = ps.getPose();
}

// vector<Point> Car::getOutline(){
//     double x, y, yaw;
//     double tail_x, tail_y, head_x, head_y;
//     vector<double> tail_l, tail_r;
//     vector<double> head_l, head_r;

//     x = pose[0];
//     y = pose[1];
//     yaw = pose[2];

//     tail_x = x - cos(yaw) * tail_length;
//     tail_y = y - sin(yaw) * tail_length;
//     tail_l.push_back(tail_x + cos(yaw + M_PI_2) * width / 2.0);
//     tail_l.push_back(tail_y + sin(yaw + M_PI_2) * width / 2.0);
//     tail_r.push_back(tail_x + cos(yaw - M_PI_2) * width / 2.0);
//     tail_r.push_back(tail_y + sin(yaw - M_PI_2) * width / 2.0);

//     head_x = x + cos(yaw) * head_length;
//     head_y = y + sin(yaw) * head_length;
//     head_l.push_back(head_x + cos(yaw + M_PI_2) * width / 2.0);
//     head_l.push_back(head_y + sin(yaw + M_PI_2) * width / 2.0);
//     head_r.push_back(head_x + cos(yaw - M_PI_2) * width / 2.0);
//     head_r.push_back(head_y + sin(yaw - M_PI_2) * width / 2.0);

//     vector<Point> outline;
//     outline.push_back(tail_l);
//     outline.push_back(tail_r);
//     outline.push_back(head_r);
//     outline.push_back(head_l);
//     outline.push_back(tail_l);
//     return outline;
// }

vector<point> Car::getCorner(){
    double x, y, yaw;
    double tail_x, tail_y, head_x, head_y;
    
    x = pose[0];
    y = pose[1];
    yaw = pose[2];

    vector<point> corners;
    tail_x = x - cos(yaw) * tail_length;
    tail_y = y - sin(yaw) * tail_length;
    corners.push_back(point(tail_x + cos(yaw + M_PI_2) * width / 2.0, tail_y + sin(yaw + M_PI_2) * width / 2.0));
    corners.push_back(point(tail_x + cos(yaw - M_PI_2) * width / 2.0, tail_y + sin(yaw - M_PI_2) * width / 2.0));
    head_x = x + cos(yaw) * (head_length + 1.0+0.2*hypot(poseState.vx,poseState.vy));    ///
    head_y = y + sin(yaw) * (head_length + 1.0+0.2*hypot(poseState.vx,poseState.vy));    ///
    corners.push_back(point(head_x + cos(yaw - M_PI_2) * width / 2.0, head_y + sin(yaw - M_PI_2) * width / 2.0));
    corners.push_back(point(head_x + cos(yaw + M_PI_2) * width / 2.0, head_y + sin(yaw + M_PI_2) * width / 2.0));
    
    return corners;
}


PoseState Car::simulate( double accel, double steer, double dt ){
    PoseState transformed = poseState.transform(poseState.getPose()); //poseState.getPose() == pose
    //double curvature = (1/steering_ratio)*steer*rear_length/(front_length*front_length + rear_length*rear_length);
    double curvature = (1/steering_ratio)*steer/(front_length + rear_length + understeer_gradient*sq(transformed.vx));
    // transformed.x, transformed.y == 0
    // transformed.vy ==? 0110101
    transformed.vy *= pow(0.5,dt);  //?????? ?????????
    
    transformed.ax = accel;
    transformed.x += transformed.vx*dt + 0.5*transformed.ax*dt*dt;
    transformed.vx += transformed.ax*dt;
    
    transformed.ay = curvature*transformed.vx*transformed.vx;
    transformed.y += transformed.vy*dt + 0.5*transformed.ay*dt*dt;
    transformed.vy += transformed.ay*dt;
    
    transformed.yaw += curvature*transformed.x;
    transformed.yaw = remainder(transformed.yaw,2*M_PI);
    transformed.yawrate = curvature*transformed.vx;

    return transformed.transform(poseState.getPose(), true);
}

vector<double> Car::findAccelSteer( double accel, double curvature ){
    vector<double> accelSteer;
    PoseState transformed = poseState.transform(poseState.getPose()); //poseState.getPose() == pose
    accelSteer.push_back(accel);
    //accelSteer.push_back(steering_ratio*curvature*(front_length*front_length + rear_length*rear_length)/rear_length);
    accelSteer.push_back(steering_ratio*curvature*(front_length + rear_length + understeer_gradient*sq(transformed.vx)));
    if(transformed.vx<0){
        accelSteer[1]*=0.5;
        ROS_ERROR("reverse");
    }
    return accelSteer;
}