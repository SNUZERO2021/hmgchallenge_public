#include <iostream>
#include <fstream>
#include <iomanip>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>


#include "hmg_utils/Object.h"
#include "hmg_utils/ObjectArray.h"
#include "hmg_utils/PredictionArray.h"
#include "hmg_utils/NearestSegment.h"
#include "hmg_utils/SegmentArray.h"
#include "hmg_utils/PredictionService.h"
#include "hmg_utils/Box.h"
#include "hmg_utils/BoxArray.h"
#include "hmg_utils/PredictionTrackerTime.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/Float32.h"
#include "hmg_utils/Targets.h"
#include "hellocm_msgs/VehicleInfo.h"
#include "hmg_utils/ConvexHull.h"
#include "hmg_utils/ConvexHullArray.h"
#include "hmg_utils/PedestrianArray.h"
#include "hmg_utils/ObjectId.h"
#include "vehicle_detection/tracker_output.h"


#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/ModelCoefficients.h>
#include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>

#include <opencv2/opencv.hpp>

#include "GlobalMap.h"
#include "ObjectSensor.h"
#include "Lidar.h"
#include "cubic_spline_2d.h"
#include "utils.h"

using namespace std;

typedef pair<int, int> pii;
typedef pair<double, double> pdd;
typedef hmg_utils::Object Object;
typedef pair<Object, Object> poo;
typedef long long ll;

struct Cost{
    double last_seq;
    double cost;

    Cost(){}
    Cost(int seq){
        cost = 1.0;
        last_seq = seq;
    }

    bool update(int cur_seq, bool hit){
        if(hit) cost = 1.0 + pow(0.8, (double)(cur_seq-last_seq)) * cost;
        else cost = pow(0.8, (double)(cur_seq-last_seq)) * cost;
        
        last_seq = cur_seq;
        if(cost > 1.5) return true;
        return false;
    }
};

// global map
GlobalMap global_map_;

// mode
bool mode;

// debug & visualize
bool debug;
bool time_debug;
bool visualize;

// dynamic obstacle configuration
double dt_;
int maxn_;
double Cs_; //offset coefficient for s
double Cl_; //offset coefficient for l
double theta_threshold_;
double l_threshold_;
double vl_limit_;

// static obstacle configuration
double err_theta_;
double edge_threshold_;
int cluster_threshold_;
double plane_tolerance_;
double dynamic_tolerance_;
double area_ratio_threshold_;
double convex_hull_clustering_threshold_;
double cost_time_threshold_;
int cost_valid_threshold_;
double dynamic_deleted_threshold_;

// map configuration
const int LENGTH = 401;
const int WIDTH = 401;
int origin_x_;
int origin_y_;
pii origin_;
double resolution_;

// vehicle configuration
double CAR_W;
double CAR_L;
double CAR_X;
double CAR_Y;
double CAR_Z;

// object sensor configuration
double object_sensor_X;
double object_sensor_Y;
double object_sensor_Z;
double object_sensor_ROLL;
double object_sensor_PITCH;
double object_sensor_YAW;

// lidar1 configuration
double lidar_1_X;
double lidar_1_Y;
double lidar_1_Z;
double lidar_1_ROLL;
double lidar_1_PITCH;
double lidar_1_YAW;

// lidar2 configuration
double lidar_2_X;
double lidar_2_Y;
double lidar_2_Z;
double lidar_2_ROLL;
double lidar_2_PITCH;
double lidar_2_YAW;


class PredictionTracker{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_object_;
    // ros::Subscriber sub_target_;
    ros::Subscriber sub_lidar1_;
    ros::Subscriber sub_lidar2_;
    ros::Publisher pub_object_;
    ros::Publisher pub_prediction_;
    ros::Publisher pub_obstacles_;
    ros::Publisher pub_pedestrians_;
    ros::Publisher pub_elapsed_time_;
    ros::Publisher pub_id_;
    stringstream segment_info_path_;
    stringstream segment_map_path_;
    stringstream height_map_path_;
    ObjectSensor object_sensor_;
    Lidar lidar1_;
    Lidar lidar2_;
    //unordered_map<ll, vector<double>> global_cost_map_;
    unordered_map<ll, Cost> global_cost_map_;
    tf::Transform odom_;
    double x_;
    double y_;
    double yaw_;
    hmg_utils::PredictionTrackerTime time_msg_;
    vector<pii> edge[LENGTH][WIDTH];
    int voxel_map[LENGTH][WIDTH];
    vector<hmg_utils::Object> prv_objects_;
    vector<hmg_utils::Object> cur_objects_;
    double prv_time_;
    double cur_time_;
    int seq_;
    int cur_segment_;
    int nxt_segment_;
    
    public:
    PredictionTracker(){
        pub_object_ = nh_.advertise<vehicle_detection::tracker_output>("/tracker_output",1);
        pub_prediction_ = nh_.advertise<hmg_utils::PredictionArray>("/prediction_array",1);
        pub_obstacles_ = nh_.advertise<hmg_utils::ConvexHullArray>("/static_obstacles",1);
        pub_pedestrians_ = nh_.advertise<hmg_utils::PedestrianArray>("/pedestrian_array", 1);
        pub_elapsed_time_ = nh_.advertise<hmg_utils::PredictionTrackerTime>("/prediction_tracker_time", 1);
        pub_id_ = nh_.advertise<hmg_utils::ObjectId>("/object_id", 1);
        sub_ = nh_.subscribe<vehicle_detection::tracker_output>("/tracker_output", 1, &PredictionTracker::callback, this);
        //sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/Odometry", 1, &PredictionTracker::callback_odom, this);
        sub_odom_ = nh_.subscribe<hellocm_msgs::VehicleInfo>("/filtered_vehicleinfo",1,&PredictionTracker::callback_odom, this);
        if(mode) sub_object_ = nh_.subscribe<hellocm_msgs::ObjectSensor>("/Object_Sensor_front", 1, &ObjectSensor::callback, &object_sensor_);
        sub_lidar1_ = nh_.subscribe<sensor_msgs::PointCloud>("/pointcloud/vlp", 1, &Lidar::callback, &lidar1_);
        sub_lidar2_ = nh_.subscribe<sensor_msgs::PointCloud>("/pointcloud/os1", 1, &Lidar::callback, &lidar2_);
        //sub_target_ = nh_.subscribe<hmg_utils::Targets>("/targets", 1, &PredictionTracker::callback_target, this);
        object_sensor_ = ObjectSensor(object_sensor_X-CAR_X, object_sensor_Y-CAR_Y, object_sensor_Z-CAR_Z, object_sensor_ROLL*M_PI/180.0, object_sensor_PITCH*M_PI/180.0, object_sensor_YAW*M_PI/180.0);
        object_sensor_.setBoardCaster(&sub_object_, &pub_object_);
        lidar1_ = Lidar(lidar_1_X-CAR_X, lidar_1_Y-CAR_Y, lidar_1_Z-CAR_Z, lidar_1_ROLL*M_PI/180.0, lidar_1_PITCH*M_PI/180.0, lidar_1_YAW*M_PI/180.0);
        lidar2_ = Lidar(lidar_2_X-CAR_X, lidar_2_Y-CAR_Y, lidar_2_Z-CAR_Z, lidar_2_ROLL*M_PI/180.0, lidar_2_PITCH*M_PI/180.0, lidar_2_YAW*M_PI/180.0);
        segment_info_path_ << ros::package::getPath("global_map") << "/config/segment_info.txt";
        segment_map_path_ << ros::package::getPath("global_map") << "/config/segment_map.txt";
        height_map_path_ << ros::package::getPath("global_map") << "/config/height_map.txt";
        time_check(PREDICTION_TRACKER, SET_DEBUG, "", time_debug);
        seq_ = 0;
        loadMap();
        buildEdge();
    }

    // void callback_odom(const nav_msgs::Odometry::ConstPtr& msg){
    //     //ROS_WARN("callback odom");
    //     geometry_msgs::Quaternion q = msg->pose.pose.orientation;
    //     geometry_msgs::Point p = msg->pose.pose.position;
    //     tf::Transform odom = tf::Transform(tf::Quaternion(q.x,q.y,q.z,q.w),tf::Vector3(p.x,p.y,p.z));
    //     tf::Vector3 vel = tf::Vector3(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    //     tf::Vector3 ang = tf::Vector3(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    //     object_sensor_.update(odom, vel, ang);
    // }

    void callback_odom(const hellocm_msgs::VehicleInfo::ConstPtr& msg){
        tf::Quaternion q;
        q.setRPY(msg->roll, msg->pitch, msg->yaw);
        x_ = msg->x[0];
        y_ = msg->x[1];
        yaw_ = msg->yaw;
        odom_ = tf::Transform(q, tf::Vector3(msg->x[0],msg->x[1],msg->x[2]));
        tf::Vector3 vel = tf::Vector3(msg->v[0],msg->v[1],msg->v[2]);
        tf::Vector3 ang = tf::Vector3(0.0,0.0,msg->yawrate);
        object_sensor_.update(odom_, vel, ang);
    }

    // void callback_target(const hmg_utils::Targets::ConstPtr& msg){
    //     cur_segment_ = msg->current_segment;
    //     nxt_segment_ = msg->next_segment;
    // }

    void loadMap(){
        int width_ = 24300; //xmax - xmin = 2430
        int height_ = 21400; //ymax - ymin = 2140
        double a1,b1,c1,a2,b2,c2; // (a1,b1,c1) = (1,0,1350), (a2,b2,c2)=(0,1,1500)
        a1 = 10.0;
        b1 = 0;
        c1 = 13500;
        a2 = 0;
        b2 = 10.0;
        c2 = 15000;
        global_map_ = GlobalMap(a1, b1,c1,a2,b2,c2,width_, height_);
        global_map_.loadSegmentInfo(segment_info_path_.str());
        global_map_.loadSegmentMap(segment_map_path_.str());
        global_map_.loadHeightMap(height_map_path_.str());
    }

    void buildEdge(){
        int n = int(ceil(edge_threshold_));
        for(int i=0;i<LENGTH;i++){
            for(int j=0;j<WIDTH;j++){
                for(int p = i-n;p<=i+n;p++){
                    for(int q = j-n;q<=j+n;q++){
                        if (p==i && q == j) continue;
                        if(p<0||p>=LENGTH||q<0||q>=WIDTH) continue;
                        if(hypot(double(i-p),double(j-q))>double(edge_threshold_)) continue;
                        edge[i][j].push_back({p,q});
                    }                    
                }
            }
        }
    }

    double sigmoid(double x, double limit){
        if(limit < 0) ROS_ERROR("limit of sigmoid is negative value!! please check it.");
        return (abs(x)<limit)?x:x*limit/abs(x);
    }

    CubicSpline2D getSegment(int id){
        return global_map_.segment[id];
    }

    pii getPixel(double x, double y){
        int px = int(round((x/resolution_)))+origin_.first;
        int py = int(round((y/resolution_)))+origin_.second;
        return {px, py};
    }

    tf::Vector3 OdomToGlobal(const pcl::PointXYZ &p){
        tf::Transform ref_T = odom_ * tf::Transform(object_sensor_.id, tf::Vector3(p.x, p.y, p.z));
        return ref_T.getOrigin();
    }

    point OdomToGlobal(const pdd &p){
        tf::Transform ref_T = odom_ * tf::Transform(object_sensor_.id, tf::Vector3(p.first, p.second, 0));
        tf::Vector3 rt = ref_T.getOrigin();
        return point(rt.x(), rt.y());
    }

    pii GlobalToPixel(const point &p){
        return {int((p.x-x_)/resolution_),int((p.y-y_)/resolution_)};
    }

    vector<ConvexHull> clustering(){
        // clustering in voxel grid map
        vector<vector<pii>> clusters;
        bool visited[LENGTH][WIDTH] = {false,};

        for(int i=0;i<LENGTH;i++){
            for(int j=0;j<WIDTH;j++){
                if(visited[i][j]) continue;
                if(voxel_map[i][j]==0){
                    visited[i][j] = true;
                    continue;
                }
                vector<pii> cluster;
                int count = 0;
                int pixel_count = 0;
                queue<pii> q;
                q.push({i,j});
                while(!q.empty()){
                    int size = q.size();
                    for(int t=0;t<size;t++){
                        pii cur = q.front();
                        q.pop();
                        if(visited[cur.first][cur.second]) continue;
                        visited[cur.first][cur.second] = true;
                        cluster.push_back(cur);
                        count += voxel_map[cur.first][cur.second];
                        pixel_count ++;
                        for(pii next : edge[cur.first][cur.second]){
                            if(voxel_map[next.first][next.second]>0 && !visited[next.first][next.second]) q.push(next);
                        }
                    }
                }
                if(count > cluster_threshold_) clusters.push_back(cluster);
            }
        }
        
        // generate convex hull
        vector<ConvexHull> convex_array;
        for(vector<pii> cluster : clusters){
            vector<point> p;
            for(pii pixel : cluster){
                p.push_back(OdomToGlobal({resolution_*(pixel.first-origin_.first+0.5),resolution_*(pixel.second-origin_.second+0.5)}));
                p.push_back(OdomToGlobal({resolution_*(pixel.first-origin_.first+0.5),resolution_*(pixel.second-origin_.second-0.5)}));
                p.push_back(OdomToGlobal({resolution_*(pixel.first-origin_.first-0.5),resolution_*(pixel.second-origin_.second+0.5)}));
                p.push_back(OdomToGlobal({resolution_*(pixel.first-origin_.first-0.5),resolution_*(pixel.second-origin_.second-0.5)}));
            }
            convex_array.push_back(ConvexHull(p));
        }

        // convex hull clustering
        vector<vector<int>> adj(convex_array.size());
        for(int i=0;i<convex_array.size();i++){
            for(int j=i+1;j<convex_array.size();j++){
                if(gjk(convex_array[i].p, convex_array[j].p)<convex_hull_clustering_threshold_){
                    adj[i].push_back(j);
                    adj[j].push_back(i);
                }
            }
        }

        // convex hull union
        vector<bool> convex_visited(convex_array.size(), false);
        vector<ConvexHull> obstacles;
        for(int i=0;i<convex_array.size();i++){
            if(convex_visited[i]) continue;
            vector<ConvexHull> cluster;
            queue<int> q;
            q.push(i);
            while(!q.empty()){
                int cur = q.front();
                q.pop();
                if(convex_visited[cur]) continue;
                cluster.push_back(convex_array[cur]);
                convex_visited[cur] =  true;
                for(int next : adj[cur]){
                    if(convex_visited[next]) continue;
                    q.push(next);
                }
            }
            obstacles.push_back(UnionConvexHull(cluster));
        }

        return obstacles;
    }

    vector<hmg_utils::Prediction> predict(const Object &prv, const Object &cur){
        ROS_INFO("[PREIDCTION] START PREDICT FUNCTION!!");
        vector<hmg_utils::Prediction> rt;
        vector<int> index_array = global_map_.getSegmentArray(cur.position.x, cur.position.y);
        vector<bool> conflict_array;
        for(int i=0;i<index_array.size();i++) conflict_array.push_back(false);


        /* filtered by theta, when object is on intersection segments */
        for(int i=0;i<index_array.size();i++){
            int index = index_array[i];
            if(!global_map_.segment[index].get_edge()[CURRENT_IS_INTER]) continue;
            double yaw = global_map_.segment[index].calc_yaw(global_map_.segment[index].find_s(cur.position.x,cur.position.y,0,false));
            if(abs(yaw-cur.position.theta)>theta_threshold_) conflict_array[i] = true;
        }


        // /* filtering conflict segment */
        // for(int i =0;i<index_array.size();i++){
        //     int index = index_array[i];
        //     if (conflict_array[i]) continue;
        //     for(int x : global_map_.segment[cur_segment].conflict_segment){
        //         if(x==index){conflict_array[i]=true; break;}
        //     }
        //     if (conflict_array[i]) continue;
        //     for(int x : global_map_.segment[nxt_segment].conflict_segment){
        //         if(x==index){conflict_array[i]=true; break;}
        //     }
        // }


        /* we need to add segment filtering part */

        /*
        // TO DO
        // TO DO
        // TO DO
        */

        PoseState ps;
        ps.x = cur.position.x;
        ps.y = cur.position.y;
        ps.yaw = cur.position.theta;
        ps.vx = cur.position.vx;
        ps.vy = cur.position.vy;
        ps.yawrate = (cur.position.theta - prv.position.theta) / (cur_time_-prv_time_);
        ps.ax = (cur.position.vx - prv.position.vx) / (cur_time_-prv_time_);
        ps.ay = (cur.position.vy - prv.position.vy) / (cur_time_-prv_time_);

        for(int i =0;i<index_array.size();i++){
            int index = index_array[i];
            if(conflict_array[i]) continue;
            CubicSpline2D segment = global_map_.segment[index];
            double max_s = segment.get_length();
            SLState sl = segment.transform(ps);
            if(abs(sl.l) > l_threshold_) continue;
            sl.ddl = 0; // ASSUME l-direction movement has constant-velocity.
            sl.dl = sigmoid(sl.dl, vl_limit_);
            hmg_utils::Prediction cur_prediction;
            vector<double> cur_smin;
            vector<double> cur_smax;
            vector<double> cur_lmin;
            vector<double> cur_lmax;
            vector<double> cur_s;
            vector<double> cur_l;
            vector<double> cur_v;
            vector<double> cur_a;
            vector<double> cur_tm;
            double ns, nds, nl, ndl, t;
            double pns, pnds, pt; // p means 'previous
            pns = sl.s;
            pnds = sl.ds;
            pt = 0.0;
            cur_prediction.segment_index = index;
            cur_prediction.object_info = cur.object_info;
            for(int cnt = 0; cnt < maxn_; cnt++){
                t = cnt * dt_;
                ns = max<double>(pns, sl.s + sl.ds * t + 0.5*sl.dds*t*t);
                nds = max<double>(0,sl.ds + sl.dds*t);
                nl = sl.l + sl.dl*t + 0.5*sl.ddl*t*t;
                ndl = sl.dl + sl.ddl*t;
                
                /* calculate prediction values */
                double smin, smax, lmin, lmax, v;
                smin = min<double>(max<double>(0, ns - Cs_*nds*t), max<double>(0, ns + Cs_*nds*t));
                smax = max<double>(max<double>(0, ns - Cs_*nds*t), max<double>(0, ns + Cs_*nds*t));
                lmin = min<double>(nl - Cl_*ndl*t, nl + Cl_*ndl*t);
                lmax = max<double>(nl - Cl_*ndl*t, nl + Cl_*ndl*t);
                v = nds;

                /* if s_min is larger than the length of segment, break */
                if(smin>max_s) break;

                /* s-value has upper boundary(s_max) */
                smin = min<double>(smin, max_s);
                smax = min<double>(smax, max_s);

                /* add prediction value */
                cur_smin.push_back(smin);
                cur_smax.push_back(smax);
                cur_lmin.push_back(lmin);
                cur_lmax.push_back(lmax);
                cur_s.push_back(ns);
                cur_l.push_back(nl);
                cur_v.push_back(v);
                cur_tm.push_back(cur_time_+t);

                if(cnt==0) cur_a.push_back(sl.dds);
                else cur_a.push_back((nds-pnds)/(t-pt));

                /* update previous value */
                pns = ns;
                pnds =nds;
                pt = t;
            }

            /* if size is 0, continue */
            if(cur_tm.size()==0) continue;

            cur_prediction.s_min = cur_smin;
            cur_prediction.s_max = cur_smax;
            cur_prediction.l_min = cur_lmin;
            cur_prediction.l_max = cur_lmax;
            cur_prediction.s = cur_s;
            cur_prediction.l = cur_l;
            cur_prediction.v = cur_v;
            cur_prediction.a = cur_a;
            cur_prediction.t = cur_tm;

            rt.push_back(cur_prediction);
        }
        return rt;
    }


    hmg_utils::Pedestrian predict_pedestrian(const Object &prv, const Object &cur){
        double x = cur.position.x;
        double y = cur.position.y;
        double vx = cur.position.vx;
        double vy = cur.position.vy;

        vector<double> cur_tm;
        vector<double> cur_x;
        vector<double> cur_y;
        vector<double> cur_vx;
        vector<double> cur_vy;

        for(int cnt = 0;cnt<maxn_;cnt++){
            double t = cnt * dt_;
            double nx = x + vx * t;
            double ny = y + vy * t;
            cur_tm.push_back(cur_time_ + t);
            cur_x.push_back(nx);
            cur_y.push_back(ny);
            cur_vx.push_back(vx);
            cur_vy.push_back(vy);
        }

        hmg_utils::Pedestrian rt;
        rt.object_info = cur.object_info;
        rt.t = cur_tm;
        rt.x = cur_x;
        rt.y = cur_y;
        rt.vx = cur_vx;
        rt.vy = cur_vy;

        return rt;
    }


    void calculate_static_obstacle(){
        clock_t begin = clock();
        clock_t begin_preprocessing = clock();

        // calculate dynamic obstacles convex hull
        for(int i=0;i<LENGTH;i++){
            for(int j=0;j<WIDTH;j++) voxel_map[i][j] = 0;
        }

        vector<ConvexHull> dynamic_obstacles;
        for(hmg_utils::Object object : cur_objects_){
            if(object.object_info.type >3) continue;
            double x,y,yaw,l,w,theta;
            x = object.position.x;
            y = object.position.y;
            yaw = object.position.theta;
            theta = atan2(object.object_info.width, object.object_info.length);
            l = max<double>(object.object_info.length, hypot(object.object_info.length,object.object_info.width)*cos(theta-err_theta_))+2.0;
            w = max<double>(object.object_info.width, hypot(object.object_info.length,object.object_info.width)*sin(theta+err_theta_))+0.3;

            vector<point> p;
            p.push_back(point(x+l/2*cos(yaw)-w/2*sin(yaw),y+l/2*sin(yaw)+w/2*cos(yaw)));
            p.push_back(point(x+l/2*cos(yaw)+w/2*sin(yaw),y+l/2*sin(yaw)-w/2*cos(yaw)));
            p.push_back(point(x-l/2*cos(yaw)+w/2*sin(yaw),y-l/2*sin(yaw)-w/2*cos(yaw)));
            p.push_back(point(x-l/2*cos(yaw)-w/2*sin(yaw),y-l/2*sin(yaw)+w/2*cos(yaw)));

            dynamic_obstacles.push_back(ConvexHull(p));
        }
        
        vector<point> vehicle_p;
        double x,y,yaw,l,w,theta;
        tf::Vector3 vehicle_position = OdomToGlobal(pcl::PointXYZ(CAR_L/2-2.5226,0,0));
        x = vehicle_position.x();
        y = vehicle_position.y();
        yaw = yaw_;
        theta = atan2(CAR_W, CAR_L);
        l = max<double>(CAR_L, hypot(CAR_W,CAR_L)*cos(theta-err_theta_));
        w = max<double>(CAR_W, hypot(CAR_W,CAR_L)*sin(theta+err_theta_));

        vehicle_p.push_back(point(x+l/2*cos(yaw)-w/2*sin(yaw),y+l/2*sin(yaw)+w/2*cos(yaw)));
        vehicle_p.push_back(point(x+l/2*cos(yaw)+w/2*sin(yaw),y+l/2*sin(yaw)-w/2*cos(yaw)));
        vehicle_p.push_back(point(x-l/2*cos(yaw)+w/2*sin(yaw),y-l/2*sin(yaw)-w/2*cos(yaw)));
        vehicle_p.push_back(point(x-l/2*cos(yaw)-w/2*sin(yaw),y-l/2*sin(yaw)+w/2*cos(yaw)));
        
        dynamic_obstacles.push_back(ConvexHull(vehicle_p));


        //merge point clouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr clouds (new pcl::PointCloud<pcl::PointXYZ>), 
                                        filtered_clouds (new pcl::PointCloud<pcl::PointXYZ>);
        *clouds += lidar1_.get_clouds();
        *clouds += lidar2_.get_clouds();

        if(visualize){
            int cv_w = 501;
            int cv_h = 501;
            cv::Mat cv_image(cv_w, cv_h, CV_8UC3, cv::Scalar(255,255,255));

            for(pcl::PointXYZ p : clouds->points){
                int px,py;
                px = p.x / resolution_;
                py = p.y / resolution_;
                cv::circle(cv_image, cv::Point(px+cv_w/2, -py+cv_h/2), 1, cv::Scalar(0,0,0));

            }
            cv::resize(cv_image, cv_image, cv::Size(200,200));
            cv::imshow("before filtering", cv_image);
            cv::waitKey(10);
        }

        

        for(pcl::PointXYZ p : clouds->points){
            tf::Vector3 cur = OdomToGlobal(p);
            vector<int> segments = global_map_.getSegmentArray(cur.x(), cur.y());
            
            // out of road
            if(segments.size() == 0) continue;
            
            // remove plane
            int index = segments[0];
            double h = global_map_.getHeight(cur.x(), cur.y());
            if(cur.z() - h < plane_tolerance_) continue;

            pii pixel = getPixel(p.x, p.y);
            if(pixel.first < 0 || pixel.first >= LENGTH || pixel.second<0 || pixel.second>=WIDTH) continue;
            voxel_map[pixel.first][pixel.second] ++;
        }
        clock_t end_preprocessing = clock();
        clock_t begin_clustering_raw = clock();
        time_msg_.preprocessing = (double)(end_preprocessing - begin_preprocessing)/CLOCKS_PER_SEC;
        clock_t end = clock();
        ROS_INFO("filtering point clouds elapsed time :  %lf", (double)(end-begin)/CLOCKS_PER_SEC);


        /* test1
        draw image with dynamic obstacles and point clouds
        point clouds are remains after  filtering out of road and ground
        */
        if(visualize){
            int cv_w = 501;
            int cv_h = 501;
            cv::Mat cv_image(cv_w, cv_h, CV_8UC3, cv::Scalar(255,255,255));

            // draw dynamic obstacles convex hulls
            // int tm = dynamic_obstacles.size();
            // for(int t = 0; t < tm; t++){
            //     int np = dynamic_obstacles[t].p.size();
            //     for(int j=0;j<np;j++){
            //         int xs = (dynamic_obstacles[t].p[j].x - x_)*10 + cv_w/2;
            //         int ys = -(dynamic_obstacles[t].p[j].y - y_)*10 + cv_h/2;
            //         int xf = (dynamic_obstacles[t].p[(j+1)%np].x - x_)*10 + cv_w/2;
            //         int yf = -(dynamic_obstacles[t].p[(j+1)%np].y - y_)*10 + cv_h/2;
            //         cv::line(cv_image, cv::Point(xs,ys), cv::Point(xf,yf), cv::Scalar(255, 0, 0), 2);
            //     }
            // }     
            // draw point cloud
            for(int i=0;i<LENGTH;i++){
                for(int j=0;j<WIDTH;j++){
                    if(voxel_map[i][j]==0) continue;
                    pii pos = GlobalToPixel(OdomToGlobal({resolution_*double(i-origin_.first), resolution_*double(j-origin_.second)}));
                    cv::circle(cv_image, cv::Point(pos.first+cv_w/2, -pos.second+cv_h/2), 3, cv::Scalar(0,0,0));
                }
            }   
            cv::resize(cv_image, cv_image, cv::Size(400,400));
            cv::imshow("dynamic bounding box", cv_image);
            cv::waitKey(10);
        }
        

        begin = clock();
        // clustering
        vector<ConvexHull> obstacles = clustering();
        if(debug) ROS_INFO("[PREDICTION] clustering raw point clouds");

        clock_t end_clustering_raw = clock();
        clock_t begin_remove_dynamic = clock();
        time_msg_.clustering_raw_data = (double)(end_clustering_raw - begin_clustering_raw)/CLOCKS_PER_SEC;

        // filtering obstacle convex hull with dynamic obstacles info
        for(ConvexHull obstacle : dynamic_obstacles){
            int index = -1;
            double dist = INF;
            double area = obstacle.getArea();
            for(int i=0;i<obstacles.size();i++){
                if(!obstacles[i].valid) continue;
                if(obstacles[i].getArea() > area_ratio_threshold_ * area) continue;
                double d = gjk(obstacles[i].p,obstacle.p);
                if(d<dynamic_tolerance_) {
                    if(debug) ROS_WARN("[PREDICTION] HIT!! dynamic obstacle will be invalid");
                    if(debug) ROS_INFO("[PREDICTION] AREA RATIO : %.2lf", obstacles[i].getArea()/area);
                    obstacles[i].valid = false;
                }
                // double d = gjk(obstacles[i].p,obstacle.p);
                // if(d>dynamic_tolerance_) continue;
                // if(d>dist) continue;
                // if(obstacles[i].getArea() > area_ratio_threshold_ * area) continue;
                // dist = d;
                // index = i;
            }
            // if(index == -1)continue;
            // obstacles[index].valid = false;
        }

        if(visualize){
            int cv_w = 501;
            int cv_h = 501;
            cv::Mat cv_image_after(cv_w, cv_h, CV_8UC3, cv::Scalar(255,255,255));

            // draw dynamic obstacles convex hulls
            int tm = dynamic_obstacles.size();
            for(int t = 0; t < tm; t++){
                int np = dynamic_obstacles[t].p.size();
                for(int j=0;j<np;j++){
                    int xs = (dynamic_obstacles[t].p[j].x - x_)*10 + cv_w/2;
                    int ys = -(dynamic_obstacles[t].p[j].y - y_)*10 + cv_h/2;
                    int xf = (dynamic_obstacles[t].p[(j+1)%np].x - x_)*10 + cv_w/2;
                    int yf = -(dynamic_obstacles[t].p[(j+1)%np].y - y_)*10 + cv_h/2;
                    cv::line(cv_image_after, cv::Point(xs,ys), cv::Point(xf,yf), cv::Scalar(255, 0, 0), 2);
                }
            }

            int n = obstacles.size();
            for(int i=0;i<n;i++){
                int np = obstacles[i].p.size();
                if(obstacles[i].valid) {
                    for(int j=0;j<np;j++){
                    int xs = (obstacles[i].p[j].x - x_)*10 + cv_w/2;
                    int ys = -(obstacles[i].p[j].y - y_)*10 + cv_h/2;
                    int xf = (obstacles[i].p[(j+1)%np].x - x_)*10 + cv_w/2;
                    int yf = -(obstacles[i].p[(j+1)%np].y - y_)*10 + cv_h/2;
                    cv::line(cv_image_after, cv::Point(xs,ys), cv::Point(xf,yf), cv::Scalar(0,255,0), 2);
                    }
                }   
                else{
                    for(int j=0;j<np;j++){
                    int xs = (obstacles[i].p[j].x - x_)*10 + cv_w/2;
                    int ys = -(obstacles[i].p[j].y - y_)*10 + cv_h/2;
                    int xf = (obstacles[i].p[(j+1)%np].x - x_)*10 + cv_w/2;
                    int yf = -(obstacles[i].p[(j+1)%np].y - y_)*10 + cv_h/2;
                    cv::line(cv_image_after, cv::Point(xs,ys), cv::Point(xf,yf), cv::Scalar(0,0,255), 2);
                    }
                }
            }
            cv::resize(cv_image_after, cv_image_after, cv::Size(400,400));
            cv::imshow("clustering result", cv_image_after);
            cv::waitKey(10);
        }
        
        if(debug) ROS_INFO("[PREDICTION] remove dynamic obstacles from convex hull clusters");
        clock_t end_remove_dynamic = clock();
        clock_t begin_voxel_reconstruction = clock();
        time_msg_.remove_dynamic_obstacles = (double)(end_remove_dynamic - begin_remove_dynamic)/CLOCKS_PER_SEC;

        if(debug){
            int sum = 0;
            int cnt = 0;
            for(const ConvexHull obstacle : obstacles){
               if(!obstacle.valid) continue;
               sum += obstacle.size;
               cnt ++;
               cout << "# of vertex : " << obstacle.size << endl;
            }
            cout << "# of obstacles : " << cnt << endl;
            cout << "# of vertices : " << sum << endl;
            cout << endl;
        }
        // update global obstacle cost map and voxel grid map
        int cnt = 0;
        for(int i = 0;i < LENGTH;i++){
            for(int j = 0; j < WIDTH; j++){
                point global_point = OdomToGlobal({resolution_*(i-origin_.first), resolution_*(j-origin_.second)});
                bool check = false;
                for(const ConvexHull &obstacle : obstacles){
                    if(!obstacle.valid) continue;
                    if(isIncludeConvexHull(obstacle, global_point)){
                        check = true;
                        break;
                    }
                }
                ll px = ll(global_point.x / resolution_);
                ll py = ll(global_point.y / resolution_);
                ll hash = px * HASH + py;
                unordered_map<ll, Cost>::iterator cost = global_cost_map_.find(hash);

                if(cost == global_cost_map_.end()){
                    if(check){
                        global_cost_map_.insert({hash, Cost(seq_)});
                    }
                    voxel_map[i][j] = 0;
                }
                else{
                    if(cost->second.update(seq_, check)) voxel_map[i][j] = 20;
                    else voxel_map[i][j] = 0;
                }
                // vector<double> v = global_cost_map_[hash];
                // if(v.size() == 0){
                //     v.push_back(cur_time_);
                //     global_cost_map_[hash] = v;
                //     voxel_map[i][j] = 0;
                // }
                // else{
                //     if(abs(v.back()-cur_time_)>EPS){
                //         vector<double> u;
                //         for(double t : v){
                //             if(cur_time_-t < cost_time_threshold_) u.push_back(t);
                //         }
                //         v = u;
                //         v.push_back(cur_time_);
                //     } 
                //     global_cost_map_[hash] = v;
                //     if(v.size()<cost_valid_threshold_) {
                //         voxel_map[i][j] = 0;
                //     }
                //     else {
                //         voxel_map[i][j] = 20;
                //         cnt ++;
                //     }
                // }
            }
        }
        if(debug) ROS_INFO("[PREDICTION] # OF OBSTACLE COST : %d",cnt);

        clock_t end_voxel_reconstruction = clock();
        clock_t begin_clustering_static = clock();
        time_msg_.voxel_reconstruction = (double)(end_voxel_reconstruction - begin_voxel_reconstruction)/CLOCKS_PER_SEC;


        // clustering
        vector<ConvexHull> new_obstacles = clustering();

        // generate static obstacle msg and publish
        vector<hmg_utils::ConvexHull> new_static_obstacles;
        vector<bool> dynamic_valid(cur_objects_.size(), true);
        vector<hmg_utils::Object> new_objects;
        for(int i=0;i<new_obstacles.size();i++){
            for(int j=0;j<cur_objects_.size();j++){
                if(cur_objects_[j].object_info.type == 0) continue;
                vector<point> v;
                v.push_back(point(cur_objects_[j].position.x, cur_objects_[j].position.y));
                if(gjk(v, new_obstacles[i].p) < dynamic_deleted_threshold_) {
                    if(debug) ROS_ERROR("[PREDICTION] Remove Pedestrian");
                    //dynamic_valid[j] = false;
                }
            }

            hmg_utils::ConvexHull cvh_msg;
            vector<geometry_msgs::Point> new_points;
            cvh_msg.count = new_obstacles[i].p.size();
            for(point pt : new_obstacles[i].p){
                geometry_msgs::Point np;
                np.x = pt.x;
                np.y = pt.y;
                np.z = 0.0;
                new_points.push_back(np);
            }
            cvh_msg.points = new_points;
            new_static_obstacles.push_back(cvh_msg);
        }

        for(int i=0;i<cur_objects_.size();i++) {
            if(dynamic_valid[i]) new_objects.push_back(cur_objects_[i]);
        }
        cur_objects_ = new_objects;
        if(debug) ROS_INFO("[PREDICTION] # OF STATIC OBSTACLES : %d", int(new_static_obstacles.size()));

        hmg_utils::ConvexHullArray rt;
        rt.obstacles = new_static_obstacles;
        end = clock();
        clock_t end_clustering_static = clock();
        time_msg_.clustering_static_obstacles = (double)(end_clustering_static - begin_clustering_static)/CLOCKS_PER_SEC;

        ROS_INFO("clustering elapsed time :  %lf", (double)(end-begin)/CLOCKS_PER_SEC);
        pub_obstacles_.publish(rt);
    }

    hmg_utils::ObjectArray TrackerToObjectArray(const vehicle_detection::tracker_output::ConstPtr& msg){
        hmg_utils::ObjectArray rt;
        rt.header = msg->header;
        double t = msg->header.stamp.toSec();

        vector<hmg_utils::Object> objects;
        for(vehicle_detection::tracking_object object : msg->data){
            hmg_utils::Object obj;
            
            obj.object_info.id = to_string(object.obj_num);
            obj.object_info.type = object.obj_class;
            obj.object_info.width = object.w;
            obj.object_info.length = object.h;
            
            obj.position.t = t;
            obj.position.x = object.x;
            obj.position.y = object.y;
            obj.position.theta = object.yaw;
            obj.position.omega = object.yaw_rate;
            obj.position.vx = object.x_d;
            obj.position.vy = object.y_d;
            obj.position.ax = object.x_dd;
            obj.position.ay = object.y_dd;

            objects.push_back(obj);
        }
        rt.objects = objects;
        return rt;
    }

    void callback(const vehicle_detection::tracker_output::ConstPtr& tracker_msg){
        seq_ ++;
        clock_t begin_total = clock();
        clock_t begin_static = clock();
        time_check(PREDICTION_TRACKER, START);
        hmg_utils::ObjectArray msg = TrackerToObjectArray(tracker_msg);
        if(debug) ROS_INFO("[PREDICTION] # of objects : %d",msg.objects.size());
        hmg_utils::PredictionArray car_predictions;
        hmg_utils::PedestrianArray pedestrian_predictions;
        //update current info
        cur_time_ = msg.header.stamp.toSec();
        cur_objects_ = msg.objects;

        hmg_utils::ObjectId rt_id;
        vector<string> cur_id;
        vector<string> prv_id;
        for(Object cur_obj : cur_objects_){
            cur_id.push_back(cur_obj.object_info.id);
        }
        for(Object prv_obj : prv_objects_){
            prv_id.push_back(prv_obj.object_info.id);
        }
        rt_id.prv = prv_id;
        rt_id.cur = cur_id;
        pub_id_.publish(rt_id);
            

        if(debug){
            printf("--------current objects--------\n");
            for(Object cur_obj : cur_objects_){
                printf("%s\n", cur_obj.object_info.id.c_str());
            }
            printf("\n\n");

            printf("--------previous objects--------\n");
            for(Object prv_obj : prv_objects_){
                printf("%s\n", prv_obj.object_info.id.c_str());
            }
            printf("\n\n");
        }
        

        calculate_static_obstacle();
        clock_t end_static = clock();
        clock_t begin_dynamic = clock();
        time_msg_.static_obstacles_detection = (double)(end_static - begin_static)/CLOCKS_PER_SEC;
        time_check(PREDICTION_TRACKER, CHECK, "detect static obstacle");
        //extract objects which appear in previous and current states
        vector<poo> objects;
        for(Object cur_obj : cur_objects_){
            for(Object prv_obj : prv_objects_){
                if(cur_obj.object_info.id.compare(prv_obj.object_info.id)==0){
                    if(debug) ROS_INFO("OBJECT MATCHING!! : %s", cur_obj.object_info.id.c_str());
                    objects.push_back({prv_obj,cur_obj});
                    break;
                }
            }
        }
        time_check(PREDICTION_TRACKER, CHECK, "extract predictable dynamics obstacles");
        //get prediction for each valid object
        vector<hmg_utils::Prediction> result_car;
        vector<hmg_utils::Pedestrian> result_pedestrian;
        for(poo obj : objects){
            if(obj.first.object_info.type==0){
                vector<hmg_utils::Prediction> rt = predict(obj.first, obj.second);
                for(hmg_utils::Prediction pd : rt) result_car.push_back(pd);
            }
            else if(obj.first.object_info.type<4) {
                if(debug) ROS_ERROR("[PREDICTION] PEDESTRIAN PREDICTION");
                hmg_utils::Pedestrian rt = predict_pedestrian(obj.first, obj.second);
                result_pedestrian.push_back(rt);
            }
        }

        clock_t end_dynamic = clock();
        time_msg_.dynamic_obstacles_prediction = (double)(end_dynamic - begin_dynamic)/CLOCKS_PER_SEC;
        time_check(PREDICTION_TRACKER, CHECK, "predict dynamic obstacles");
        //update prediction array
        car_predictions.predictions = result_car;
        car_predictions.header = msg.header;
        pub_prediction_.publish(car_predictions);
        
        pedestrian_predictions.predictions = result_pedestrian;
        pedestrian_predictions.header = msg.header;
        pub_pedestrians_.publish(pedestrian_predictions);

        clock_t end_total = clock();
        time_msg_.total_time = (double)(end_total - begin_total)/CLOCKS_PER_SEC;
        pub_elapsed_time_.publish(time_msg_);

        //replace previous state to current state
        prv_time_ = cur_time_;
        prv_objects_ = cur_objects_;
        time_check(PREDICTION_TRACKER, CHECK, "end prediction tracker");
    }
};





int main(int argc, char **argv){
    ros::init(argc, argv, "prediction_tracker");

    // mode
    ros::param::get("/ideal", mode); // if true, object sensor turn on!!

    // debug & visualize
    ros::param::get("/prediction_debug", debug);
    ros::param::get("/prediction_time_debug", time_debug);
    ros::param::get("/prediction_visualize", visualize);
    // dynamic obstacle configuration
    ros::param::get("/dt", dt_);
    ros::param::get("/maxn", maxn_);
    ros::param::get("/Cs", Cs_);
    ros::param::get("/Cl", Cl_);
    ros::param::get("/theta_threshold", theta_threshold_);
    ros::param::get("/l_threshold", l_threshold_);
    ros::param::get("/vl_limit", vl_limit_);

    // static obstacle configuration
    ros::param::get("/err_theta", err_theta_); err_theta_ = err_theta_ * M_PI /180.0;
    ros::param::get("/edge_threshold", edge_threshold_);
    ros::param::get("/cluster_threshold", cluster_threshold_);
    ros::param::get("/plane_tolerance", plane_tolerance_);
    ros::param::get("/dynamic_tolerance", dynamic_tolerance_);
    ros::param::get("/area_ratio_threshold", area_ratio_threshold_);
    ros::param::get("/convex_hull_clustering_threshold", convex_hull_clustering_threshold_);
    ros::param::get("/cost_time_threshold", cost_time_threshold_);
    ros::param::get("/cost_valid_threshold", cost_valid_threshold_);
    ros::param::get("/dynamic_deleted_threshold", dynamic_deleted_threshold_);

    // map configuration
    ros::param::get("/origin_x", origin_x_);
    ros::param::get("/origin_y", origin_y_); origin_ = {origin_x_, origin_y_};
    ros::param::get("/resolution", resolution_);

    // vehicle configuration
    ros::param::get("/CAR_W", CAR_W);
    ros::param::get("/CAR_L", CAR_L);
    ros::param::get("/CAR_X", CAR_X);
    ros::param::get("/CAR_Y", CAR_Y);
    ros::param::get("/CAR_Z", CAR_Z);

    // object sensor configuration
    ros::param::get("/object_sensor_X", object_sensor_X);
    ros::param::get("/object_sensor_Y", object_sensor_Y);
    ros::param::get("/object_sensor_Z", object_sensor_Z);
    ros::param::get("/object_sensor_ROLL", object_sensor_ROLL);
    ros::param::get("/object_sensor_PITCH", object_sensor_PITCH);
    ros::param::get("/object_sensor_YAW", object_sensor_YAW);

    // lidar1 configuration
    ros::param::get("/lidar_1_X", lidar_1_X);
    ros::param::get("/lidar_1_Y", lidar_1_Y);
    ros::param::get("/lidar_1_Z", lidar_1_Z);
    ros::param::get("/lidar_1_ROLL", lidar_1_ROLL);
    ros::param::get("/lidar_1_PITCH", lidar_1_PITCH);
    ros::param::get("/lidar_1_YAW", lidar_1_YAW);

    // lidar2 configuration
    ros::param::get("/lidar_2_X", lidar_2_X);
    ros::param::get("/lidar_2_Y", lidar_2_Y);
    ros::param::get("/lidar_2_Z", lidar_2_Z);
    ros::param::get("/lidar_2_ROLL",lidar_2_ROLL);
    ros::param::get("/lidar_2_PITCH", lidar_2_PITCH);
    ros::param::get("/lidar_2_YAW", lidar_2_YAW);
    

    PredictionTracker prediction_tracker; 
    ros::spin();
}
