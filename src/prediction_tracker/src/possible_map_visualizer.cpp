// #include <iostream>
// #include <fstream>
// #include <iomanip>
// #include <math.h>
// #include <stdlib.h>
// #include <ctime>
// #include <set>
// #include <string>
// #include <vector>
// #include <queue>


// #include "ros/ros.h"
// #include "ros/time.h"
// #include "ros/package.h"


// #include "GlobalMap.h"
// #include "cubic_spline_2d.h"

// #include "opencv2/opencv.hpp"

// using namespace std;
// using namespace cv;


// struct rgb{
//     public:
//     int r,g,b;
//     rgb():rgb(0,0,0){}
//     rgb(int x, int y, int z):r(x),g(y),b(z){}
// };

// GlobalMap global_map_;
    

// int main(int argc, char **argv){
//     ros::init(argc, argv, "possible_map_visualizer");
//     srand((unsigned int)time(NULL));
//     stringstream segment_info_path_;
//     stringstream segment_map_path_;
//     stringstream segment_nearest_path_;
//     stringstream save_path_;
//     int width_ = 24300; //xmax - xmin = 2430
//     int height_ = 21400; //ymax - ymin = 2140
//     double a1,b1,c1,a2,b2,c2; // (a1,b1,c1) = (1,0,1350), (a2,b2,c2)=(0,1,1500)

//     a1 = 10;
//     b1 = 0;
//     c1 = 13500;
//     a2 = 0;
//     b2 = 10;
//     c2 = 15000;
    
//     segment_info_path_ << ros::package::getPath("prediction_tracker") << "/config/segment_info.txt";
//     segment_map_path_ << ros::package::getPath("prediction_tracker") << "/config/segment_map.txt";
//     //segment_nearest_path_ << ros::package::getPath("prediction_tracker")<<"/config/segment_nearest.txt";
//     save_path_ << ros::package::getPath("prediction_tracker") <<"/config/map.png";

    
//     global_map_ = GlobalMap(a1, b1,c1,a2,b2,c2,width_,height_);
    
//     global_map_.loadSegmentInfo(segment_info_path_.str());
//     global_map_.loadSegmentMap(segment_map_path_.str());
//     //global_map_.loadSegmentNearest(segment_nearest_path_.str());
//     int cnt = global_map_.segment.size();
    
//     vector<rgb> colors;
//     for(int i=0;i<cnt;i++){
//         int r,g,b;
//         r = rand();
//         g = rand();
//         b = rand();
//         colors.push_back(rgb(r,g,b));
//     }
//     int minx,maxx, miny, maxy;
//     minx = 6000;
//     maxx = 12000;
//     miny = 15000;
//     maxy = 21000;

//     Mat img = Mat(maxy-miny, maxx-minx, CV_8UC3, Scalar(255,255,255));
    
//     cout << "success to load global map" <<endl;
//     for(int i=0;i<global_map_.global_segment_map.size();i++){
//         int px = global_map_.global_segment_map[i].x;
//         int py = global_map_.global_segment_map[i].y;
//         vector<int> v = global_map_.global_segment_map[i].segments;
        
//         if(px<minx || px>=maxx || py < miny || py >= maxy) continue;
//         px -= minx;
//         py -= miny;
//         int r=0,g=0,b=0;
        
//         for(int idx:v){
//             r += colors[idx].r;
//             g += colors[idx].g;
//             b += colors[idx].b;
//         }
//         r %= 256;
//         g %= 256;
//         b %= 256;
//         img.at<Vec3b>(maxy - miny - py,px)[0] = 255-r;
//         img.at<Vec3b>(maxy - miny - py,px)[1] = 255-g;
//         img.at<Vec3b>(maxy - miny - py,px)[2] = 255-b;
//     }
    
//     for(int i=0;i<cnt;i++){
//         vector<double> x = global_map_.segment[i].get_x();
//         vector<double> y = global_map_.segment[i].get_y();
//         for(int j=0;j<x.size();j++){
//             pii p = global_map_.XYToPixel(x[j],y[j]);
//             if(p.first < minx || p.first >= maxx || p.second < miny || p.second >= maxy) continue;
//             p.first -= minx;
//             p.second -= miny;
//             cout << p.first <<" "<<p.second << endl;
//             circle(img, cv::Point(p.first,maxy - miny - p.second), 3, Scalar(255,0,0),-1);
//             /*img.at<Vec3b>(p.second, p.first)[0] = 255;
//             img.at<Vec3b>(p.second, p.first)[1] = 0;
//             img.at<Vec3b>(p.second, p.first)[2] = 0;
//             */
//         }
//     }

//     cout <<"completed to build img" << endl;
//     imwrite(save_path_.str(),img);
//     cout <<"completed to save img"<<endl;
// }