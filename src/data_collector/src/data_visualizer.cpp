#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include <queue>
#include <unistd.h>


#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "hellocm_msgs/ObjectSensor.h"
#include "hellocm_msgs/ObjectSensorObj.h"
#include "hellocm_msgs/ObservPoint.h"


#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"


#include "Lidar.h"
#include "utils.h"


using namespace std;
using namespace cv;

typedef pair<int, int> pii;
typedef pair<double, double> pdd;

struct oo{
    public:
    double x,y,yaw,w,l;
    oo():oo(0,0,0,0,0){}
    oo(double a, double b, double c,double d, double e):x(a),y(b),yaw(c),w(d),l(e){}
};

vector<pdd> point_clouds;
vector<oo> objects;

string get_string(int n, int d){
    if(n>pow(10,d)) {
        ROS_ERROR("N is too large... can't change into string");
        return "";
    }
    if(n==0) return "0000";
    int digit = int(log10(n));
    string rt = "";
    for(int i=0;i<d-1-digit;i++) rt += "0";
    rt += to_string(n);
    return rt;
}

bool valid(int x, int y){
    if(x<0 || x>=10000||y<0||y>=10000) return false;
    return true;
}

void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees)
{
    cv::Scalar color = cv::Scalar(0, 0, 255.0); // red

    // Create the rotated rectangle
    cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);

    // We take the edges that OpenCV calculated for us
    cv::Point2f vertices2f[4];
    rotatedRectangle.points(vertices2f);

    // Convert them so we can use them in a fillConvexPoly
    cv::Point vertices[4];    
    for(int i = 0; i < 4; ++i){
        vertices[i] = vertices2f[i];
    }

    // Now we can fill the rotated rectangle with our specified color
    cv::fillConvexPoly(image,
                       vertices,
                       4,
                       color);
}

int main(int argc, char **argv){
    cout << "please type in the scenario number and time in 0.1seconds scale" << endl;
    cout << "for example '1 12' for scenario 1 in 1.2seconds" << endl;
    ros::init(argc, argv, "data_visualizer");
    
    double resolution = 0.1;
    int scenario = 1;
    int time = 10;
    const char delimiter = ' ';
    cin >> scenario >> time;
  //  cout << "debug2" << endl;
    
    stringstream lidar_path;
    stringstream object_path;
    lidar_path << ros::package::getPath("data_collector") << "/data/Scenario" << get_string(scenario,2) << "/Lidar/Lidar_Scenario" << get_string(scenario,2) << "_" << get_string(time, 4) << ".txt";        
    object_path << ros::package::getPath("data_collector") << "/data/Scenario" << get_string(scenario,2) << "/Object/Object_Scenario" << get_string(scenario,2) << "_" << get_string(time, 4) << ".txt";
        
  //  cout << "debug3" << endl;
    /* lidar data decoding */
   // cout << "debug4" << endl;
    bool check = true;
    int n = 0; // # of pointcloud
    string in_line;
    ifstream in(lidar_path.str());
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> tokens;
        while(getline(ss,token,delimiter)) tokens.push_back(token);
        if(check) {n = stoi(tokens[0]); check = false;}
        else point_clouds.push_back({stod(tokens[0]), stod(tokens[1])});
    }


    /* object info decoding */
   // cout << "debug5" << endl;
    check = true;
    n = 0;
    string in_line2;
    int state = 0;
    ifstream in2(object_path.str());
    oo obj;
    while(getline(in2, in_line2)){
        stringstream ss(in_line2);
        string token;
        vector<string> tokens;
        while(getline(ss, token, delimiter)) tokens.push_back(token);
        if(check){n=stoi(tokens[0]); check = false; state=0;}
        else{
            if(state==0){
                obj.l = stod(tokens[2]);
                obj.w = stod(tokens[3]);   
                state=1;
            }
            else if(state==1){
                obj.x = stod(tokens[0]);
                obj.y = stod(tokens[1]);
                state=2;
            }
            else if(state==2){
                obj.yaw = stod(tokens[2]);
                state =3;
            }
            else if(state==3){
                objects.push_back(obj);
                obj = oo();
                state = 0;
            }
        }
    }

    Mat img = Mat(10000,10000,CV_8UC3, Scalar(255,255,255));
    Mat img_Lidar = Mat(10000,10000,CV_8UC3, Scalar(255,255,255));
    Mat img_Object = Mat(10000,10000,CV_8UC3, Scalar(255,255,255));


    /* Object coloring(red) */
    cout << "# of objects " << objects.size() <<endl;
    for(oo object : objects){
        int nx = 5000 - int(object.y/resolution);
        int ny = 5000 - int(object.x/resolution);
        if(!valid(nx, ny)) {cout << "invalid" <<endl; continue;}
        DrawRotatedRectangle(img,cv::Point(nx, ny), Size(int(object.l/resolution), int(object.w/resolution)), 360 - object.yaw*180/M_PI); 
        
    
        DrawRotatedRectangle(img_Object,cv::Point(nx, ny), Size(int(object.l/resolution), int(object.w/resolution)), 360 - object.yaw*180/M_PI);        
    }
    
    cv::Vec3b LidarColor(255,0,0);
    /* Lidar coloring(blue) */
    cout << "# of point clouds" << point_clouds.size()<< endl;
    for(pdd point : point_clouds){
        int nx = 5000 - int(point.first/resolution);
        int ny = 5000 - int(point.second/resolution);
        if(!valid(nx, ny)) continue;
        img_Lidar.at<Vec3b>(cv::Point(nx,ny)) = LidarColor;
       
    
        img.at<Vec3b>(cv::Point(nx,ny)) = LidarColor;
    }
   // cout << "debug7" << endl;
    

    
   // cout << "debug9" << endl;
    
    //namedWindow("img");
    imshow("img", img);
    imshow("img_Lidar", img_Lidar);
    imshow("img_Object", img_Object);
    stringstream save_path;
    stringstream save_path_Lidar;
    stringstream save_path_Object;
    save_path << ros::package::getPath("data_collector") << "/image/Combined/Combined_sample" << scenario <<"_"<<time<< ".png";
    save_path_Lidar << ros::package::getPath("data_collector") << "/image/Lidar/Lidar_sample" << scenario <<"_"<<time<< ".png";
    save_path_Object << ros::package::getPath("data_collector") << "/image/Object/Object_sample" << scenario <<"_"<<time<< ".png";


    imwrite(save_path.str(), img);
    imwrite(save_path_Lidar.str(), img_Lidar);
    imwrite(save_path_Object.str(), img_Object);

}
