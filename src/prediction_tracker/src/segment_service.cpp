#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include <queue>


#include "hmg_utils/ObjectArray.h"
#include "hmg_utils/PredictionArray.h"
#include "hmg_utils/NearestSegment.h"
#include "hmg_utils/SegmentArray.h"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"


#include "GlobalMap.h"
#include "History.h"
#include "cubic_spline_2d.h"

GlobalMap global_map_;
    

bool callback_segment(hmg_utils::SegmentArray::Request &req,
                      hmg_utils::SegmentArray::Response &res){
    res.segments = global_map_.getSegmentArray(req.x, req.y);
    if(res.segments.size()==0) return false;
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "segment_service");
    ros::NodeHandle nh_;
    ros::ServiceServer service_segment_ = nh_.advertiseService("get_segment", callback_segment);
    stringstream segment_info_path_;
    stringstream segment_map_path_;
    int width_ = 24300; //xmax - xmin = 2430
    int height_ = 21400; //ymax - ymin = 2140
    double a1,b1,c1,a2,b2,c2; // (a1,b1,c1) = (1,0,1350), (a2,b2,c2)=(0,1,1500)
    a1 = 10.0;
    b1 = 0;
    c1 = 13500;
    a2 = 0;
    b2 = 10.0;
    c2 = 15000;
    
    segment_info_path_ << ros::package::getPath("global_map") << "/config/segment_info.txt";
    segment_map_path_ << ros::package::getPath("global_map") << "/config/segment_map.txt";
    
    global_map_ = GlobalMap(a1, b1,c1,a2,b2,c2,width_, height_);
    global_map_.loadSegmentInfo(segment_info_path_.str());
    global_map_.loadSegmentMap(segment_map_path_.str());
    ros::spin();
}
