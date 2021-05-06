#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <ctime>
#include <set>
#include <string>
#include <vector>
#include <queue>


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"


#include "GlobalMap.h"
#include "cubic_spline_2d.h"

#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


struct rgb{
    public:
    int r,g,b;
    rgb():rgb(0,0,0){}
    rgb(int x, int y, int z):r(x),g(y),b(z){}
};

GlobalMap global_map;


int main(int argc, char **argv){
    //////////////////////////////// HyperParameter /////////////////////////////////
    double l_range = 2.5;    // 세그먼트의 l 범위
    double s_range = 33.0;   // 교차로 직전 s 범위
    /////////////////////////////////////////////////////////////////////////////////


    ros::init(argc, argv, "build_intersection_map");
    stringstream segment_info_path;
    stringstream segment_map_path;
    stringstream save_path;
    int width_ = 24300; //xmax - xmin = 2430
    int height_ = 21400; //ymax - ymin = 2140
    double a1,b1,c1,a2,b2,c2; // (a1,b1,c1) = (1,0,1350), (a2,b2,c2)=(0,1,1500)

    a1 = 10;
    b1 = 0;
    c1 = 13500;
    a2 = 0;
    b2 = 10;
    c2 = 15000;
    
    segment_info_path << ros::package::getPath("global_map") << "/config/segment_info.txt";
    segment_map_path << ros::package::getPath("global_map") << "/config/segment_map.txt";
    save_path << ros::package::getPath("global_map") <<"/config/intersection_map.png";

    
    global_map = GlobalMap(a1, b1,c1,a2,b2,c2,width_,height_);
    
    global_map.loadSegmentInfo(segment_info_path.str());
    global_map.loadSegmentMap(segment_map_path.str());
    int cnt = global_map.segment.size();
    printf("success to load global map size %d\n", cnt);
    
    clock_t begin = clock();

    // build intersection map
    stringstream intersection_map_path;
    intersection_map_path << ros::package::getPath("global_map") <<"/config/intersection_map.txt";
    ofstream intersection_map_out(intersection_map_path.str());

    int width = 24300;
    int height = 21400;
    double step = 0.1;
    double rstep = 0.05;
    vector<vector<bool>> intersection_map(width, vector<bool>(height, false));

    for(int index = 0; index < global_map.segment.size(); index++){
        if(index%100 == 0) printf("intersection map segment index %d\n", index);
        CubicSpline2D &cubic = global_map.segment[index];
        bool cur_inter = cubic.get_edge()[CURRENT_IS_INTER];
        bool next_inter = cubic.get_edge()[NEXT_IS_INTER];
        vector<double> boundary = cubic.get_boundary();
        if(cur_inter){
            double s = boundary[1];
            PoseState ps;
            pii pixel;
            double theta;
            while(s<boundary[2]){
                ps = cubic.sl_to_xy(SLState(s,0));
                theta = cubic.calc_yaw(s);
                for(int px = -3; px<=3;px++){
                    for(int py = -(int)(l_range/rstep);py<=(int)(l_range/rstep);py++){
                        double x = ps.x + px*rstep*cos(theta) - py*rstep*sin(theta);
                        double y = ps.y + px*rstep*sin(theta) + py*rstep*cos(theta);
                        pii cur = global_map.XYToPixel(x,y);
                        intersection_map[cur.first][cur.second] = true;
                    }
                }
                s += step;
            }
        }
        else if(next_inter){
            double s = boundary[2] - s_range;
            PoseState ps;
            pii pixel;
            double theta;
            while(s<boundary[3]){
                ps = cubic.sl_to_xy(SLState(s,0));
                theta = cubic.calc_yaw(s);
                for(int px = -3; px<=3;px++){
                    for(int py = -(int)(l_range/rstep);py<=(int)(l_range/rstep);py++){
                        double x = ps.x + px*rstep*cos(theta) - py*rstep*sin(theta);
                        double y = ps.y + px*rstep*sin(theta) + py*rstep*cos(theta);
                        pii cur = global_map.XYToPixel(x,y);
                        intersection_map[cur.first][cur.second] = true;
                    }
                }
                s += step;
            }
        }
        
    }

    for(int i = 0;i<width;i++){
        for(int j=0;j<height;j++){
            if(intersection_map[i][j]==false) continue;
            intersection_map_out << to_string(i) << " "<<to_string(j);
            intersection_map_out << endl; 
        }
    }

    intersection_map_out.close();
    vector<vector<bool>>().swap(intersection_map);  

    clock_t end = clock();
    cout << "elapsed time : "<<double(end-begin)/CLOCKS_PER_SEC<<endl;
    printf("completed to build intersection map\n");

    global_map.loadIntersectionMap(intersection_map_path.str());

    printf("success to load intersection map\n");

    // draw intersection map
    rgb color(100,200,200);
    int minx,maxx, miny, maxy;
    minx = 6000;
    maxx = 24000;
    miny = 3000;
    maxy = 21000;

    // minx = 5000;
    // maxx = 7000;
    // miny = 18000;
    // maxy = 21000;

    Mat img = Mat(maxy-miny, maxx-minx, CV_8UC3, Scalar(255,255,255));

    for(int px = minx;px<maxx;px++){
        for(int py=miny;py<maxy;py++){
            pdd xy = global_map.PixelToXY(px, py);
            bool b = global_map.isIntersection(xy.first, xy.second);
            if(b) {
                img.at<Vec3b>(maxy - 1 - miny - (py - miny), px - minx)[0] = color.r;
                img.at<Vec3b>(maxy - 1 - miny - (py - miny), px - minx)[1] = color.g;
                img.at<Vec3b>(maxy - 1 - miny - (py - miny), px - minx)[2] = color.b;
            }
        }
    }
    printf("completed to build img\n");
    imwrite(save_path.str(),img);
    printf("completed to save img\n");
}