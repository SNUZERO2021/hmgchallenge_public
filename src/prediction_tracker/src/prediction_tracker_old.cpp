#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include <queue>


#include "prediction_tracker/ObjectArray.h"
#include "prediction_tracker/PredictionArray.h"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"


#include "GlobalMap.h"
#include "History.h"
#include "cubic_spline_2d.h"


typedef pair<double, double> pdd;

using namespace std;

struct HistoryCompare{
    bool operator()(History const &h1, History const &h2){
        return h1.object_info.id.compare(h2.object_info.id) < 0;
    }
};

class PredictionTracker{
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double cur_time_;
    double threshold_;
    int prediction_threshold_;
    set<History,HistoryCompare> history_;
    GlobalMap global_map_;
    //stringstream segment_info_path_;
    //stringstream segment_map_path_;

    public:
    PredictionTracker(){
        pub_ = nh_.advertise<prediction_tracker::PredictionArray>("/prediction_array", 2);
	    sub_ = nh_.subscribe<prediction_tracker::ObjectArray>("/object_array", 1, &PredictionTracker::callback, this);
        threshold_ = 1.00;
        prediction_threshold_ = 5;
        loadMap();
    }

    void loadMap(){
        //global_map_.loadSegmentMap(segment_map_path_);
        //global_map_.loadSegmentInfo(segment_info_path_);
    }

    PoseState getPoseState(prediction_tracker::Position p){
        PoseState rt;
        rt.x = p.x;
        rt.y = p.y;
        rt.yaw = p.theta;
        rt.vx = p.x*cos(p.theta);
        rt.vy = p.y*sin(p.theta);
        rt.yawrate = p.omega;
        rt.ax = p.accel * cos(p.theta);
        rt.ay = p.accel * sin(p.theta);
        return rt;
    }

    prediction_tracker::Prediction predict(History h, CubicSpline2D seg){
        double t0 = h.last_time;
        /*vector<pdd> pos;
        for(prediction_tracker::Position u : h){
            PoseState ps = getPoseState(u);
            SLState st = seg.transform(ps);
            //linear fitting, quadratic fitting with s and l, then choose one which has smaller error.
        }*/

        return prediction_tracker::Prediction();
    }

    void callback(const prediction_tracker::ObjectArray::ConstPtr& msg){
        cur_time_ = ros::Time::now().toSec();

        // update history with object array
        // search object in history
        for(prediction_tracker::Object obj : msg->objects){
            set<History, HistoryCompare>::iterator iter = history_.begin();
            while(iter!=history_.end()){ // check if cur id is same as object id
                History cur_h = *iter;
                if(cur_h.check_id(obj.object_info.id)) break;
                iter++;
            }
            if(iter == history_.end()){
                //create new history
                History h(obj.object_info);
                h.updateSegmentArray(global_map_.getSegmentArray(obj.position.x, obj.position.y));
                h.insert(obj.position);
                history_.insert(h);
            }
            else{
                //insert object info into history
                //update history
                History cur_h = *iter;
                cur_h.insert(obj.position);
                cur_h.updateSegmentArray(global_map_.getSegmentArray(obj.position.x, obj.position.y));
            }
        }

        // erase empty history element
        set<History, HistoryCompare>::iterator iter = history_.begin();
        while(iter!=history_.end()){
            History cur_h = *iter;
            cur_h.update(cur_time_,threshold_);
            if(cur_h.size==0){
                set<History,HistoryCompare>::iterator tmp = ++iter;
                iter--;
                history_.erase(iter);
                iter = tmp;
            }
            else iter++;
        }

        prediction_tracker::PredictionArray rt;
        vector<prediction_tracker::Prediction> predictions;
        iter = history_.begin();
        while(iter!=history_.end()){
            History cur_h = *iter;
            if(cur_h.size>prediction_threshold_){
                for(int idx : cur_h.segment_array){
                    prediction_tracker::Prediction prediction = predict(cur_h, global_map_.segment.at(idx));
                }
            }
            iter++;
        }

        rt.predictions = predictions;
        pub_.publish(rt);
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "prediction_tracker");
    //clock_t begin = clock();
    PredictionTracker prediction_tracker;
    ros::spin();
}