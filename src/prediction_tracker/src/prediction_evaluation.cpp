#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include <queue>


#include "hmg_utils/Object.h"
#include "hmg_utils/ObjectArray.h"
#include "hmg_utils/PredictionArray.h"
#include "hmg_utils/NearestSegment.h"
#include "hmg_utils/SegmentArray.h"
#include "hmg_utils/PredictionService.h"
#include "hmg_utils/Box.h"
#include "hmg_utils/BoxArray.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float32.h"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include "GlobalMap.h"
#include "ObjectSensor.h"
#include "cubic_spline_2d.h"
#include "utils.h"

#define foreach BOOST_FOREACH

typedef pair<double,double> pdd;

const int MAX_SEQUENCE = 500;
const int MAX_OBJECT_NUM = 10;
const int MAX_PREDICTION = 20;
const double RADIUS = 0.1;

struct box{
    double x,y;
    pdd boundary[4];
};

struct node{
    double xg, yg;
    vector<box> boxes[MAX_PREDICTION];
    int optimal_index[MAX_PREDICTION];
    bool valid;

    node(){
        for(int i=0;i<MAX_PREDICTION;i++) optimal_index[i] = -1;
        valid = false;
    }
};

stringstream segment_info_path;
stringstream bag_file_path;
    
pdd get_xy(double s, double l, int index, GlobalMap &global_map){
    SLState sl;
    sl.s = s;
    sl.l = l;
    PoseState ps = global_map.segment[index].sl_to_xy(sl);
    return {ps.x, ps.y};
}

double get_distance(pdd a, pdd b){
    return sqrt(sq(a.first-b.first)+sq(a.second-b.second));
}

double get_ratio(pdd ref, box target, int t){
    if(t==0) return 1.0;
    double r = RADIUS*t;
    double x_min, x_max, y_min, y_max;
    x_min = ref.first - r;
    x_max = ref.first + r;
    y_min = ref.second - r;
    y_max = ref.second + r;

    vector<point> p;
    for(int i=0;i<4;i++) p.push_back(point(target.boundary[i].first,target.boundary[i].second));
    ConvexHull cv = ConvexHull(p);

    for(int i=0;i<4;i++){
        if(x_min > target.boundary[i].first) x_min = target.boundary[i].first;
        if(x_max < target.boundary[i].first) x_max = target.boundary[i].first;
        if(y_min > target.boundary[i].second) y_min = target.boundary[i].second;
        if(y_max < target.boundary[i].second) y_max = target.boundary[i].second;
    }

    int cap = 0; // # of points which is included ground truth region or target boundary
    int hat = 0; // # of points which is included ground truth region and target boundary
    double x,y;
    double step = 0.01;
    x = x_min;
    y = y_min;

    while(1){
        bool check1 = true;
        bool check2 = true;

        if(get_distance({x,y},{ref.first,ref.second})>r) check1 = false;

        int w = ccw(point(x,y),cv.p[cv.size-1],cv.p[0]);
        for(int i=0;i<cv.size-1;i++){
            if(w != ccw(point(x,y),cv.p[i],cv.p[i+1])) check2 = false; 
        }

        cap += check1||check2;
        hat += check1&&check2;
        x += step;
        if(x>x_max){
            x = x_min;
            y += step;
        }
        if(y>y_max) break;
    }

    return (double)hat/(double)cap;
}

int main(int argc, char **argv){
    ROS_INFO("START EVALUATION");
    ros::init(argc, argv, "prediction_evaluation");

    /* Load GLOBAL MAP */
    ROS_INFO("LOAD GLOBAL MAP");
    segment_info_path << ros::package::getPath("global_map") << "/config/segment_info.txt";
    int width_ = 24300; //xmax - xmin = 2430
    int height_ = 21400; //ymax - ymin = 2140
    double a1 = 10.0;
    double b1 = 0;
    double c1 = 13500;
    double a2 = 0;
    double b2 = 10.0;
    double c2 = 15000;
    GlobalMap global_map = GlobalMap(a1, b1,c1,a2,b2,c2,width_, height_);
    global_map.loadSegmentInfo(segment_info_path.str());

    /* Load Bag File */
    ROS_INFO("LOAD BAG FILE");
    bag_file_path << ros::package::getPath("prediction_tracker") << "/test.bag";
    rosbag::Bag bag(bag_file_path.str());
    rosbag::View view_object(bag, rosbag::TopicQuery("/tracker_output"));
    rosbag::View view_prediction(bag, rosbag::TopicQuery("/prediction_array"));

    /* Initialize dataset */
    ROS_INFO("INITIALIZE DATASET");
    node data[MAX_SEQUENCE][MAX_OBJECT_NUM];

    /* Decode ground truth data */
    ROS_INFO("LOAD GROUND TRUTH DATA");
    BOOST_FOREACH(rosbag::MessageInstance const msg, view_object){
        hmg_utils::ObjectArray::ConstPtr object_array = msg.instantiate<hmg_utils::ObjectArray>();
        if (object_array != NULL){
            int seq = int((object_array->header.stamp.toSec()+EPS)*10);
            if(seq>=MAX_SEQUENCE) continue;
            for(hmg_utils::Object object : object_array->objects){
                cout << "[PREDICTION_EVALUATION] " << object.object_info.id<<endl;
                int id = stoi(object.object_info.id.substr(0,4));
                if(id>=MAX_OBJECT_NUM) continue;
                data[seq][id].xg = object.position.x;
                data[seq][id].yg = object.position.y;
                data[seq][id].valid = true;
            }
        }
    }

    /* Decode prediction results */
    ROS_INFO("LOAD PREDICTION RESULTS");
    BOOST_FOREACH(rosbag::MessageInstance const msg, view_prediction){
        hmg_utils::PredictionArray::ConstPtr prediction_array = msg.instantiate<hmg_utils::PredictionArray>();
        if(prediction_array != NULL){
            int seq = int((prediction_array->header.stamp.toSec()+EPS)*10);
            if(seq>=MAX_SEQUENCE) continue;
            for(hmg_utils::Prediction prediction : prediction_array->predictions){
                int id = stoi(prediction.object_info.id.substr(0,4));
                if(id>=MAX_OBJECT_NUM) continue;
                int size = prediction.t.size();
                int index = prediction.segment_index;
                CubicSpline2D segment = global_map.segment[index];
                for(int i=0;i<size;i++){
                    int time = int((prediction.t[i]-prediction.t[0]+EPS)*10);
                    if(seq+time>=MAX_SEQUENCE) continue;

                    box b;
                    b.x = get_xy(prediction.s[i], prediction.l[i], index, global_map).first;
                    b.y = get_xy(prediction.s[i], prediction.l[i], index, global_map).second;
                    b.boundary[0] = get_xy(prediction.s_min[i], prediction.l_min[i], index, global_map);
                    b.boundary[1] = get_xy(prediction.s_min[i], prediction.l_max[i], index, global_map);
                    b.boundary[2] = get_xy(prediction.s_max[i], prediction.l_max[i], index, global_map);
                    b.boundary[3] = get_xy(prediction.s_max[i], prediction.l_min[i], index, global_map);
                    data[seq+time][id].boxes[time].push_back(b);
                }
            }
        }
    }

    bag.close();

    /* Calculate N-prediction_distance results */
    ROS_INFO("CALCULATE PREDICTION DISTANCE ACCURACY");
    vector<double> scores[MAX_PREDICTION];

    for(int i=0;i<MAX_SEQUENCE;i++){
        for(int j=0;j<MAX_OBJECT_NUM;j++){
            if(!data[i][j].valid) continue;
            for(int t=0;t<MAX_PREDICTION;t++){
                if(data[i][j].boxes[t].size()==0) continue;
                int index = -1;
                double min_d = INF;
                for(int s = 0; s<data[i][j].boxes[t].size();s++){
                    box b = data[i][j].boxes[t][s];
                    double d = get_distance({data[i][j].xg,data[i][j].yg},{b.x,b.y});
                    if(min_d>d){
                        min_d = d;
                        index = s;
                    }
                }

                if(index <0) continue;
                scores[t].push_back(min_d);
                data[i][j].optimal_index[t] = index;
            }
        }
    }
    for(int i=0;i<MAX_PREDICTION;i++){
        if(scores[i].size()==0) {
            ROS_ERROR("There is no info about %d-th prediction distance accuracy score", i);
            continue;
        }
        double sum = 0.0;
        for(double x : scores[i]) sum += x;
        ROS_INFO("%d-th prediction distance accuracy score is %lf", i, sum/scores[i].size());
    }

    /* Calculate area ratio */
    ROS_INFO("CALCULATE PREDICTION AREA ACCURACY");
    vector<double> ratio[MAX_PREDICTION];

    for(int i=0;i<MAX_SEQUENCE;i++){
        for(int j=0;j<MAX_OBJECT_NUM;j++){
            int k = i*MAX_OBJECT_NUM + j;
            ROS_INFO("%d/%d",k,MAX_SEQUENCE*MAX_OBJECT_NUM);
            if(!data[i][j].valid) continue;
            for(int t=0;t<MAX_PREDICTION;t++){
                int index = data[i][j].optimal_index[t];
                if(index<0) continue;
                ratio[t].push_back(get_ratio({data[i][j].xg,data[i][j].yg}, data[i][j].boxes[t][index], t));
            }
        }
    }
    for(int i=0;i<MAX_PREDICTION;i++){
        if(ratio[i].size()==0){
            ROS_ERROR("There is no info about %d-th prediction area accuracy score", i);
            continue;
        }
        double sum = 0.0;
        for(double x : ratio[i]) sum += x;
        ROS_INFO("%d-th prediction area accuracy score is %lf", i, sum/ratio[i].size());
    }
}