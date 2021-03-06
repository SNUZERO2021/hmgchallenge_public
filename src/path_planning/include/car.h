#ifndef FRENET_OPTIMAL_TRAJECTORY_CAR_H
#define FRENET_OPTIMAL_TRAJECTORY_CAR_H

#include "utils.h"
#include <vector>
#include <tuple>

using namespace std;

const double VEHICLE_FRONT_OVERHANG = 0.8424; // front end to front wheel
const double VEHICLE_FRONT_LENGTH = 1.37;    // front wheel to mass center
const double VEHICLE_REAR_LENGTH = 1.33; // mass center to rear wheel
const double VEHICLE_REAR_OVERHANG = 0.8326;   // real wheel to real end
const double VEHICLE_ORGIN_FROM_REAR = VEHICLE_REAR_OVERHANG + VEHICLE_REAR_LENGTH;    // vehicle orgin from vehicle rear end. orgin is center of mass
//const double VEHICLE_ORGIN_FROM_REAR = 0;    // vehicle orgin from vehicle rear end. orgin is real end.
//const double CORNERING_STIFFNESS_RATIO_OF_FRONT_TO_REAR = 1;    // front cornering stiffness / rear cornering stiffness
//const double VEHICLE_LENGTH = 4.93;
const double VEHICLE_WIDTH = 1.805;
const double STEERING_RATIO = 14.317695;
const double UNDERSTEER_GRADIENT = 0.0007808;
class Car {
 public:
    double front_length; // wheel to mass center
    double rear_length;
    //wheel_base = front_length + rear_length;
    double head_length; // end to orgin
    double tail_length;
    //total_length = head_length + tail_length;
    double width;
    double steering_ratio;
    double understeer_gradient;

    Pose pose; // x, y, yaw
    PoseState poseState; // x, y, yaw, vx, vy, yawrate, ax, ay

    Car(){
        front_length = VEHICLE_FRONT_LENGTH;
        rear_length = VEHICLE_REAR_LENGTH;
        head_length = VEHICLE_FRONT_OVERHANG + VEHICLE_FRONT_LENGTH + VEHICLE_REAR_LENGTH + VEHICLE_REAR_OVERHANG - VEHICLE_ORGIN_FROM_REAR;
        tail_length = VEHICLE_ORGIN_FROM_REAR;
        width = VEHICLE_WIDTH;
        steering_ratio = STEERING_RATIO;
        understeer_gradient = UNDERSTEER_GRADIENT;
        pose.push_back(0);
        pose.push_back(0);
        pose.push_back(0);
        setPose(pose);
    };
    Car(Pose pose_): pose(pose_) {
        front_length = VEHICLE_FRONT_LENGTH;
        rear_length = VEHICLE_REAR_LENGTH;
        head_length = VEHICLE_FRONT_OVERHANG + VEHICLE_FRONT_LENGTH + VEHICLE_REAR_LENGTH + VEHICLE_REAR_OVERHANG - VEHICLE_ORGIN_FROM_REAR;
        tail_length = VEHICLE_ORGIN_FROM_REAR;
        width = VEHICLE_WIDTH;
        steering_ratio = STEERING_RATIO;
        understeer_gradient = UNDERSTEER_GRADIENT;
        setPose(pose);
    };
    void setPose(Pose p);
    void setPose(PoseState ps);
    // vector<Point> getOutline();
    vector<point> getCorner();
    PoseState simulate( double accel, double steer, double dt );
    vector<double> findAccelSteer( double accel, double curvature );
};

#endif