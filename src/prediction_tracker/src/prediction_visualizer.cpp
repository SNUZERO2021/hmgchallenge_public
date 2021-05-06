#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include <queue>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "hmg_utils/PredictionArray.h"
#include "hmg_utils/Prediction.h"
#include "hmg_utils/ObjectInfo.h"
#include "nav_msgs/Odometry.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"


#include "GlobalMap.h"
#include "cubic_spline_2d.h"
#include "utils.h"

using namespace std;
using namespace cv;

typedef pair<double, double> pdd;
typedef pair<int, int> pii;

GlobalMap global_map_;

class PredictionVisualizer{
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    ros::Subscriber sub_odom_;
    stringstream segment_info_path_;
    Mat img;
    pdd cur_;
    int height_;
    int width_;
    double object_radius_;
    double time_;
    double resolution_;

    public:
    PredictionVisualizer(){
        pub_ = nh_.advertise<sensor_msgs::Image>("/prediction/image_raw", 2);
        sub_ = nh_.subscribe<hmg_utils::PredictionArray>("/prediction_array",1,&PredictionVisualizer::callback, this);
        sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry",1,&PredictionVisualizer::callback_odom, this);
        segment_info_path_ << ros::package::getPath("global_map") << "/config/segment_info.txt";
        resolution_ = 0.1;
        object_radius_ = 10;
        height_ = 1000;
        width_ = 1000;
        loadMap();
    }

    void loadMap(){
        global_map_.loadSegmentInfo(segment_info_path_.str());
    }

    void callback_odom(const nav_msgs::Odometry::ConstPtr& msg){
        cur_.first = msg->pose.pose.position.x;
        cur_.second = msg->pose.pose.position.y;
    }

    void DrawPrediction(hmg_utils::Prediction prediction){
        int n = prediction.t.size();
        CubicSpline2D segment = global_map_.segment[prediction.segment_index];
        if(n==0) return;
        SLState st;
        PoseState ps;
        st.s = (prediction.s_min[0]+prediction.s_max[0])/2;
        st.l = (prediction.l_min[0]+prediction.l_max[0])/2;
        ps = segment.sl_to_xy(st);
        circle(img, cv::Point(height_/2+int((ps.x-cur_.first)/resolution_),int(width_/2+(ps.y-cur_.second)/resolution_)), object_radius_*2, Scalar(255,0,0,0), -1, 8, 0);
        for(int i=1;i<n;i++){
            double alpha = (prediction.t[i]-time_)/3.0;
            
            vector<cv::Point> contour;
            
            st.s = prediction.s_min[i];
            st.l = prediction.l_min[i];
            ps = segment.sl_to_xy(st);
            contour.push_back(cv::Point(int(height_/2+(ps.x-cur_.first)/resolution_),int(width_/2+(ps.y-cur_.second)/resolution_)));
            st.s = prediction.s_min[i];
            st.l = prediction.l_max[i];
            ps = segment.sl_to_xy(st);
            contour.push_back(cv::Point(int(height_/2+(ps.x-cur_.first)/resolution_),int(width_/2+(ps.y-cur_.second)/resolution_)));
            st.s = prediction.s_max[i];
            st.l = prediction.l_max[i];
            ps = segment.sl_to_xy(st);
            contour.push_back(cv::Point(int(height_/2+(ps.x-cur_.first)/resolution_),int(width_/2+(ps.y-cur_.second)/resolution_)));
            st.s = prediction.s_max[i];
            st.l = prediction.l_min[i];
            ps = segment.sl_to_xy(st);
            contour.push_back(cv::Point(int(height_/2+(ps.x-cur_.first)/resolution_),int(width_/2+(ps.y-cur_.second)/resolution_)));
            
            for(cv::Point point : contour){
                circle(img, point, object_radius_, Scalar(255,0,0, int(alpha*255)), -1, 8, 0);
            }

            const cv::Point *pts = (const cv::Point*) Mat(contour).data;
            int npts = Mat(contour).rows;
            polylines(img, &pts, &npts, 1, false, Scalar(255, 0, 0, int(alpha*255)));
        }
    }

    void callback(const hmg_utils::PredictionArray::ConstPtr& msg){
        time_ = msg->header.stamp.toSec();
        
        img = Mat(height_, width_, CV_8UC4, Scalar(255,255,255,0));
        circle(img, cv::Point(height_/2,width_/2), object_radius_*3, Scalar(0,255,0,0), -1, 8, 0);

        for(hmg_utils::Prediction prediction : msg->predictions){
            DrawPrediction(prediction);
            // int size = prediction.t.size();
            // SLState st;
            // PoseState ps;
            // pii pixel;
            // st.s = (prediction.s_min[0]+prediction.s_max[0])/2;
            // st.l = (prediction.l_min[0]+prediction.l_max[0])/2
            // ps = segment.sl_to_xy(st);
            // pixel = global_map_.XY_To_Pixel(ps.x, ps.y);
            // circle(img, cv::Point(pixel.first, pixel.second), object_radius_, Scalar(255,0,0),-1,8,0);
            // for(int i=1;i<s;i++){
            //     st.s = (prediction.s_min[i]+prediction.s_max[i])/2;
            //     st.l = (prediction.l_min[i]+prediction.l_max[i])/2
            //     ps = segment.sl_to_xy(st);
            //     pixel = global_map_.XY_To_Pixel(ps.x, ps.y);
            //     circle(img, cv::Point(pixel.first, pixel.second), prediction_radius_, Scalar(100,100,0),-1,8,0);
            // }
        }

        sensor_msgs::Image rt;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge;
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
        img_bridge.toImageMsg(rt);
        pub_.publish(rt);

        imshow("prediction_result",img);
        waitKey(5);


        return;
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "prediction_visualizer");
    PredictionVisualizer prediction_visualizer;
    
    ros::spin();
}
