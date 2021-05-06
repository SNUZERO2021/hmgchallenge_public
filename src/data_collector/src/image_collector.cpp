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
#include "sensor_msgs/Image.h"


#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include "utils.h"


using namespace std;
using namespace cv;



class ImageCollector{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    int cnt =0;
    public:
    ImageCollector(){
        sub_ = nh_.subscribe<sensor_msgs::Image>("/color1/image_raw", 1, &ImageCollector::callback, this);
        
    }
    
    void callback(const sensor_msgs::Image::ConstPtr &msg){
        cnt++;
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        stringstream save_path_;
        save_path_ << ros::package::getPath("data_collector") << "/test/" << to_string(cnt) << ".png";
        cout << save_path_.str() << endl;
        imwrite(save_path_.str(), cv_ptr->image);
        namedWindow("please");
        imshow("please", cv_ptr->image);
        
        waitKey(5);
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "image_collector");
    ROS_INFO("START IMAGE COLLECTOR");
    ImageCollector image_collector;

    ros::spin();
}
