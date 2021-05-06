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
#include <algorithm>
#include <cstdlib> 


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"


#include "GlobalMap.h"
#include "cubic_spline_2d.h"

#include <opencv2/opencv.hpp>

using namespace std;



GlobalMap global_map_;

double alpha = 0.5;
int iter = 5;

void smooth(CubicSpline2D & csp, int index){

    int offset = 20;
    int ratio = 50;
    double minx = 1e10, maxx = -1e10, miny = 1e10, maxy = -1e10;
    vector<double> x = csp.get_x();
    vector<double> y = csp.get_y();
    vector<double> s = csp.get_s();
    vector<double> boundary = csp.get_boundary();
    vector<double> original_z = csp.get_z();
    vector<double> original_x = x;
    vector<double> original_y = y;
    int sz = csp.get_size();

    for(double _x : x){
        minx = min(minx, _x);
        maxx = max(maxx, _x);
    }
    for(double _y : y){
        miny = min(miny, _y);
        maxy = max(maxy, _y);
    }
    minx -= 20;
    maxx += 20;
    miny -= 20;
    maxy += 20;

    vector<int> boundary_index;
    for(int i = 0;i<4;i++){
        for(int j = 0;j<sz;j++){
            if(abs(s[j] - boundary[i]) < 0.5){
                boundary_index.push_back(j);
                break;
            }
        }
    }
    if(boundary_index.size() != 4){
        printf("boundary not found for csp %d", index);
        assert(false);
    }

    cv::Mat img = cv::Mat(((int)(maxy-miny))*ratio+offset,((int)(maxx-minx))*ratio+offset, CV_8UC3, cv::Scalar(0,0,0));
    for(int i=0;i<sz;i++){
        int _x = (x[i] - minx)*ratio + offset/2;
        int _y = (maxy - 1 - y[i])*ratio + offset/2;
        circle(img, cv::Point(_x, _y), 1, cv::Scalar(255,0,0), -1);        
    }

    vector<cv::Scalar> colors;
    for(int i = 0;i<iter;i++){
        colors.push_back(cv::Scalar((255*(iter-i-1))/iter,0,(255*i)/iter));
    }
    for(int cnt = 0; cnt<iter;cnt++){
        vector<int> permutation;
        for(int i = 0;i<sz;i++) permutation.push_back(i);
        random_shuffle(permutation.begin(), permutation.end());

        for(int i =0;i<sz;i++){
            int p = permutation[i];
            if(p==boundary_index[0] || p==boundary_index[1] || p==boundary_index[2] || p==boundary_index[3]) continue;

            vector<double> new_x, new_y;
            for(int j = 0;j<sz;j++){
                if(j==p) continue;
                new_x.push_back(x[j]);
                new_y.push_back(y[j]);
            }
            CubicSpline2D tmp_csp(new_x,new_y, original_z);

            PoseState ps;
            ps.x = original_x[p];
            ps.y = original_y[p];
            SLState sl = tmp_csp.transform(ps);
            sl.l *= alpha;
            ps = tmp_csp.sl_to_xy(sl);

            x[p] = ps.x;
            y[p] = ps.y;
        }   
        for(int i=0;i<sz;i++){
            int _x = (x[i] - minx)*ratio + offset/2;
            int _y = (maxy - 1 - y[i])*ratio + offset/2;
            // printf("iter %d : %d , %d\n",cnt, _x, _y);
            circle(img, cv::Point(_x, _y), 1, colors[cnt], -1);        
        }  
    }
    stringstream save_path;
    save_path << ros::package::getPath("global_map") <<"/config/smooth_test" << index  << ".png";
    //cv::namedWindow("segment");
    //cv::imshow("segment",img);
    //cv::waitKey(20000);
     cv::imwrite(save_path.str(), img);
}

int maxy = 450;
int miny = 300;
int minx = -350;
int maxx = -250;
int ratio = 50;
int offset = 50;
cv::Mat img;


void smooth2(CubicSpline2D & csp, int index){

    vector<double> x = csp.get_x();
    vector<double> y = csp.get_y();
    vector<double> s = csp.get_s();
    vector<double> boundary = csp.get_boundary();
    vector<double> original_z = csp.get_z();
    vector<double> original_x = x;
    vector<double> original_y = y;
    int sz = csp.get_size();

    vector<int> boundary_index;
    for(int i = 0;i<4;i++){
        for(int j = 0;j<sz;j++){
            if(abs(s[j] - boundary[i]) < 0.5){
                boundary_index.push_back(j);
                break;
            }
        }
    }
    if(boundary_index.size() != 4){
        printf("boundary not found for csp %d", index);
        assert(false);
    }

    
    for(int i=0;i<sz;i++){
        int _x = (x[i] - minx)*ratio + offset/2;
        int _y = (maxy - 1 - y[i])*ratio + offset/2;
        circle(img, cv::Point(_x, _y), 1, cv::Scalar(255,0,0), -1);        
    }

    vector<cv::Scalar> colors;
    for(int i = 0;i<iter;i++){
        colors.push_back(cv::Scalar((255*(iter-i-1))/iter,0,(255*i)/iter));
    }
    for(int cnt = 0; cnt<iter;cnt++){
        vector<int> permutation;
        for(int i = 0;i<sz;i++) permutation.push_back(i);
        random_shuffle(permutation.begin(), permutation.end());

        for(int i =0;i<sz;i++){
            int p = permutation[i];
            if(p==boundary_index[0] || p==boundary_index[1] || p==boundary_index[2] || p==boundary_index[3]) continue;

            vector<double> new_x, new_y;
            for(int j = 0;j<sz;j++){
                if(j==p) continue;
                new_x.push_back(x[j]);
                new_y.push_back(y[j]);
            }
            CubicSpline2D tmp_csp(new_x,new_y, original_z);

            PoseState ps;
            ps.x = original_x[p];
            ps.y = original_y[p];
            SLState sl = tmp_csp.transform(ps);
            sl.l *= alpha;
            ps = tmp_csp.sl_to_xy(sl);

            x[p] = ps.x;
            y[p] = ps.y;
        }   
        if(cnt != iter-1) continue;
        for(int i=0;i<sz;i++){
            int _x = (x[i] - minx)*ratio + offset/2;
            int _y = (maxy - 1 - y[i])*ratio + offset/2;
            // printf("iter %d : %d , %d\n",cnt, _x, _y);
            circle(img, cv::Point(_x, _y), 1, colors[cnt], -1);        
        }  
    }
    stringstream save_path;
    save_path << ros::package::getPath("global_map") <<"/config/smooth_test" << index  << ".png";
    //cv::namedWindow("segment");
    //cv::imshow("segment",img);
    //cv::waitKey(20000);
}

void smooth3(CubicSpline2D & csp, int index){
    vector<double> x = csp.get_x();
    vector<double> y = csp.get_y();
    vector<double> s = csp.get_s();
    vector<double> boundary = csp.get_boundary();
    vector<double> original_z = csp.get_z();
    vector<double> original_x = x;
    vector<double> original_y = y;
    int sz = csp.get_size();


    vector<int> boundary_index;
    for(int i = 0;i<4;i++){
        for(int j = 0;j<sz;j++){
            if(abs(s[j] - boundary[i]) < 0.5){
                boundary_index.push_back(j);
                break;
            }
        }
    }
    if(boundary_index.size() != 4){
        printf("boundary not found for csp %d", index);
        assert(false);
    }

    for(int i=0;i<sz;i++){
        int _x = (x[i] - minx)*ratio + offset/2;
        int _y = (maxy - 1 - y[i])*ratio + offset/2;
        circle(img, cv::Point(_x, _y), 1, cv::Scalar(0,150,150), -1);        
    }

    vector<cv::Scalar> colors;
    for(int i = 0;i<iter;i++){
        colors.push_back(cv::Scalar(0,150+ (155*(i+1))/iter, 150 - (150*(i+1))/iter));
    }
    for(int cnt = 0; cnt<iter;cnt++){
        vector<int> permutation;
        for(int i = 0;i<sz;i++) permutation.push_back(i);
        random_shuffle(permutation.begin(), permutation.end());

        for(int i =0;i<sz;i++){
            int p = permutation[i];
            if(p==boundary_index[0] || p==boundary_index[1] || p==boundary_index[2] || p==boundary_index[3]) continue;

            vector<double> new_x, new_y;
            for(int j = 0;j<sz;j++){
                if(j==p) continue;
                new_x.push_back(x[j]);
                new_y.push_back(y[j]);
            }
            CubicSpline2D tmp_csp(new_x,new_y, original_z);

            PoseState ps;
            ps.x = original_x[p];
            ps.y = original_y[p];
            SLState sl = tmp_csp.transform(ps);
            sl.l *= alpha;
            ps = tmp_csp.sl_to_xy(sl);

            x[p] = ps.x;
            y[p] = ps.y;
        } 
        if(cnt != iter-1) continue;
        for(int i=0;i<sz;i++){
            int _x = (x[i] - minx)*ratio + offset/2;
            int _y = (maxy - 1 - y[i])*ratio + offset/2;
            // printf("iter %d : %d , %d\n",cnt, _x, _y);
            circle(img, cv::Point(_x, _y), 1, colors[cnt], -1);        
        }  
    }
    
    //cv::namedWindow("segment");
    //cv::imshow("segment",img);
    //cv::waitKey(20000);
    
}
    

int main(int argc, char **argv){
    ros::init(argc, argv, "test_smooth");
    srand ( unsigned ( std::time(0) ) );
    stringstream segment_info_path_;
    stringstream segment_map_path_;
    stringstream segment_nearest_path_;
    stringstream save_path_;
    int width_ = 24300; //xmax - xmin = 2430
    int height_ = 21400; //ymax - ymin = 2140
    double a1,b1,c1,a2,b2,c2; // (a1,b1,c1) = (1,0,1350), (a2,b2,c2)=(0,1,1500)

    a1 = 10;
    b1 = 0;
    c1 = 13500;
    a2 = 0;
    b2 = 10;
    c2 = 15000;
    
    segment_info_path_ << ros::package::getPath("global_map") << "/config/segment_info.txt";
    segment_map_path_ << ros::package::getPath("global_map") << "/config/segment_map.txt";
    save_path_ << ros::package::getPath("global_map") <<"/config/map.png";



    
    global_map_ = GlobalMap(a1, b1,c1,a2,b2,c2,width_,height_);
    
    global_map_.loadSegmentInfo(segment_info_path_.str());
    global_map_.loadSegmentMap(segment_map_path_.str());
    int cnt = global_map_.segment.size();
    CubicSpline2D csp = global_map_.segment[21];

    smooth(csp, 21);

/*
    for(int i = 2;i<6;i++){
        alpha = 0.1 * i;
        img = cv::Mat(((int)(maxy-miny))*ratio+offset,((int)(maxx-minx))*ratio+offset, CV_8UC3, cv::Scalar(0,0,0));
        CubicSpline2D csp = global_map_.segment[21];

        smooth2(csp, 21);


        CubicSpline2D csp2 = global_map_.segment[37];

        smooth3(csp2, 37);
        stringstream save_path;
        save_path << ros::package::getPath("global_map") <<"/config/smooth_test" << i << ".png";
        cv::imwrite(save_path.str(), img);
    }
    */
   
}