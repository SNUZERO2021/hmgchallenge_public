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
#include "hmg_utils/Targets.h"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>

#include "GlobalMap.h"
#include "ObjectSensor.h"
#include "Lidar.h"
#include "cubic_spline_2d.h"
#include "utils.h"


using namespace std;


class ObstacleDetector{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_object_;
    Lidar lidar1_;
    Lidar lidar2_;
    

    public:
    ObstacleDetector(){
        sub_object_ = nh_.subscribe<hmg_utils::ObjectArray>("/object_array", 1, &ObstacleDetector::callback, this);
    }

    void callback(const hmg_utils::ObjectArray::ConstPtr& msg){
        // merge point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds (new pcl::PointCloud<pcl::PointXYZ>), 
                                        filtered_clouds (new pcl::PointCloud<pcl::PointXYZ>);
        *clouds += lidar1_.get_clouds();
        *clouds += lidar2_.get_clouds();

        
        // remove plane
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);       //(옵션) // Enable model coefficient refinement (optional).
        seg.setInputCloud (clouds);                 //입력 
        seg.setModelType (pcl::SACMODEL_PLANE);    //적용 모델  // Configure the object to look for a plane.
        seg.setMethodType (pcl::SAC_RANSAC);       //적용 방법   // Use RANSAC method.
        seg.setMaxIterations (1000);               //최대 실행 수
        seg.setDistanceThreshold (0.01); 
        
        seg.segment(*inliers, *coefficients);

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (clouds);
        extract.setIndices (inliers);
        extract.setNegative (true);//false
        extract.filter (*filtered_clouds);

        // remove object from object sensor
        

        // transform 2d voxel grid map

        // clustering

        // generate convex hull

        // publish
    }
};




int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_detector");
    //ObstacleDetector obstacle_detector;
    ros::spin();
}
