#include "ros/ros.h"
#include <iostream>
#include "hellocm_msgs/VehicleInfo.h"
#include "hellocm_msgs/GPS_Out.h"
#include "nav_msgs/Odometry.h"
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include "utils.h"
#include "proj.h"
#include "Eigen/Eigen"
#define PI 3.141592653589793238463
#define SMALL 1e-6
#define _ST 14
#define _U 0
#define _Z 11   ///
#define _ZPos 3
inline double trp(double x){
  return x*x*x;
}
using namespace Eigen;
class KalmanFusion{
// state : yaw,vx,vy,vz,yawrate,ax,ay,az,yawacc,roll,pitch, x,y,z
// observation : vehicle info (yaw,vx,vy,vz,yawrate,ax,ay,az,yawacc,roll,pitch,steer,gas,break,gear)
// 0    1   2   3   4   5   6   7   8   9   10  11  12  13
// yaw  vx  vy  vz  yr  ax  ay  az  ya  rll ptc x   y   z   : state
// yaw  vx  vy  vz  yr  ax  ay  az  ya  rll ptc slp la      : observation (vehicle info)
// x    y   z                                               : observation (pos)
 
public:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Time t_;
  Matrix<double,_ST,1> st;
  Matrix<double,_U,1> u;
  Matrix<double,_ST,_ST> P;    // P is covariance of stATE
  Matrix<double,_ST,_ST> Q;
  Matrix<double,_Z,_Z> R;
  Matrix<double,_ZPos,_ZPos> RPos;
  hellocm_msgs::VehicleInfo msg_;
  double std_accrate, std_yawaccrate, std_acczrate, std_rollpitchrate;
  double  x_std_err, v_std_err, a_std_err, yaw_std_err, yawrate_std_err, yawacc_std_err;
  double xz_std_err, vz_std_err, az_std_err, rollpitch_std_err;
  double slip_std_err, latacc_std_err;
  double front_length;
  double rear_length;
  double steering_ratio;
  double understeer_gradient;
  double slip_coeff;
  int count = 0;
  int pos_count = 0;
  KalmanFusion(){
      st.setZero();
      u.setZero();
      P.setIdentity();
      Q.setIdentity();
      R.setIdentity();
      RPos.setIdentity();
      pub_ = n_.advertise<hellocm_msgs::VehicleInfo>("filtered_vehicleinfo", 10);
      ros::param::get("/std_accrate", std_accrate);
      ros::param::get("/std_yawaccrate", std_yawaccrate);
      ros::param::get("/std_acczrate", std_acczrate);
      ros::param::get("/std_rollpitchrate", std_rollpitchrate);
      ros::param::get("/x_std_err", x_std_err);
      ros::param::get("/v_std_err", v_std_err);
      ros::param::get("/a_std_err", a_std_err);
      ros::param::get("/xz_std_err", xz_std_err);
      ros::param::get("/vz_std_err", vz_std_err);
      ros::param::get("/az_std_err", az_std_err);
      ros::param::get("/yaw_std_err", yaw_std_err);
      ros::param::get("/yawrate_std_err", yawrate_std_err);
      ros::param::get("/yawacc_std_err", yawacc_std_err);
      ros::param::get("/rollpitch_std_err", rollpitch_std_err);
      ros::param::get("/slip_std_err", slip_std_err);
      ros::param::get("/latacc_std_err", latacc_std_err);
      ros::param::get("/vehicle_front_length", front_length);
      ros::param::get("/vehicle_rear_length", rear_length);
      ros::param::get("/steering_ratio", steering_ratio);
      ros::param::get("/understeer_gradient", understeer_gradient);
      slip_coeff = understeer_gradient*front_length/(rear_length-front_length);
      P/=SMALL;
      for(int i=0;i<3;i++){
          RPos(i,i) *= sq(x_std_err);
          R(i+1,i+1) *= sq(v_std_err);
          R(i+5,i+5) *= sq(a_std_err);
      }
      R(0,0) *= sq(yaw_std_err);
      R(4,4) *= sq(yawrate_std_err);
      R(8,8) *= sq(yawacc_std_err);
      R(9,9) *= sq(rollpitch_std_err);
      R(10,10) *= sq(rollpitch_std_err);
    //   R(11,11) *= sq(slip_std_err);
    //   R(12,12) *= sq(latacc_std_err);
  }
  void Callback(const hellocm_msgs::VehicleInfo& msg){
      hellocm_msgs::VehicleInfo tmp_msg = msg;
      count++;
      if(msg.header.frame_id == "Fr1"){
        tf::Quaternion q;
        tf::Quaternion id;
        id.setRPY(0,0,0);
        q.setRPY(msg.roll, msg.pitch, msg.yaw);
        tf::Transform odom_ = tf::Transform(q, tf::Vector3(0,0,0));
        tf::Transform vel_ = tf::Transform(id, tf::Vector3(msg.v[0],msg.v[1],msg.v[2]));
        tf::Transform acc_ = tf::Transform(id, tf::Vector3(msg.a[0],msg.a[1],msg.a[2]));
        
        tf::Vector3 global_vel_ = (odom_ * vel_).getOrigin();
        tf::Vector3 global_acc_ = (odom_ * acc_).getOrigin();

        tmp_msg.v[0] = global_vel_.x();
        tmp_msg.v[1] = global_vel_.y();
        tmp_msg.v[2] = global_vel_.z();
        tmp_msg.a[0] = global_acc_.x();
        tmp_msg.a[1] = global_acc_.y();
        tmp_msg.a[2] = global_acc_.z();    
      }else{
        ROS_ERROR("[Kalman] Invalid Cooldinate : check hellocm_cmnode");
      }
      ///
      // st[0] = tmp_msg.yaw;
      // st[1] = tmp_msg.v[0];
      // st[2] = tmp_msg.v[1];
      // st[3] = tmp_msg.v[2];
      // st[4] = tmp_msg.yawrate;
      // st[5] = tmp_msg.a[0];
      // st[6] = tmp_msg.a[1];
      // st[7] = tmp_msg.a[2];
      // st[8] = tmp_msg.yawacc;
      // st[9] = tmp_msg.roll;
      // st[10] = tmp_msg.pitch;
      // if(pos_count>0){
      //     msg_ = msg;
      //     publish(msg.header.stamp);
      // }
      // return;
      ///
      predict(msg.header.stamp);
      // z = h(st)+v
      // H = dh/dst
      // v ~ N(0,R)
      Matrix<double,_Z,_ST> H;
      Matrix<double,_Z,1> z;
      double msgyaw = remainder(msg.yaw,2*PI);
    //   z << msgyaw, msg.v[0], msg.v[1], msg.v[2], msg.yawrate, msg.a[0], msg.a[1], msg.a[2], msg.yawacc, msg.roll, msg.pitch, 0, 0;
      z << msgyaw, tmp_msg.v[0], tmp_msg.v[1], tmp_msg.v[2], msg.yawrate, tmp_msg.a[0], tmp_msg.a[1], tmp_msg.a[2], msg.yawacc, msg.roll, msg.pitch;
     
      //z11 = atan2(vy,vx)-yaw - SC(-ax*sinv + ay*cosv) : slip angle error
      //z12 = (-ax*sinv + ay*cosv) - yawrate*v : lateral acceleration error
      H.setIdentity();
    //   H(11,11) = 0;
    //   H(12,12) = 0;
    //   if(hypot(st(1),st(2))>SMALL){
    //       double cosv = st(1)/hypot(st(1),st(2));
    //       double sinv = st(2)/hypot(st(1),st(2));
    //       H(11,0) = -1; // yaw -> slip
    //       H(11,5) = slip_coeff*sinv;    // ax -> slip
    //       H(11,6) = -slip_coeff*cosv;   // ay -> slip
    //       //H(11,1), H(11,2)   // vx, vy -> slip
    //       H(12,5) = -sinv; // ax -> latacc
    //       H(12,6) = cosv;  // ay -> latacc
    //       //H(12,1), H(12,2), H(12,4) // vx, vy, yawrate -> latacc
    //   }else{
    //       H(11,5) = slip_coeff*sin(st(0));
    //       H(11,6) = -slip_coeff*cos(st(0));
    //       H(12,5) = -sin(st(0));
    //       H(12,6) = cos(st(0));
    //   }
    //   H(12,4) = hypot(st(1),st(2));    // yawrate -> latacc
      Matrix<double,_Z,1> y = z-H*st;
    //   if(hypot(st(1),st(2))>SMALL){
    //       double cosv = st(1)/hypot(st(1),st(2));
    //       double sinv = st(2)/hypot(st(1),st(2));
    //       H(12,1) = ( -st(5)*sq(st(2)) + st(6)*(-st(1)*st(2)) )/ trp(hypot(st(1),st(2)));  // vx -> latacc
    //       H(12,2) = ( -st(5)*(-st(1)*st(2)) + st(6)*sq(st(1)) )/ trp(hypot(st(1),st(2)));  // vy -> latacc
    //       H(11,1) = -sinv/hypot(st(1),st(2)) - slip_coeff*H(12,1); // vx -> slip
    //       H(11,2) = cosv/hypot(st(1),st(2)) - slip_coeff*H(12,2);  // vy -> slip
    //       y(11) += atan2(st(2),st(1));
    //       H(12,1) += st(4)*cosv;  // vx -> latacc
    //       H(12,2) += st(4)*sinv ;  // vy -> latacc
    //   }
    //   y(11) = remainder(y(11),2*PI);
      y(0) = remainder(y(0),2*PI);
      Matrix<double,_Z,_Z> S = H*P*H.transpose()+R;
      Matrix<double,_ST,_Z> K = P*H.transpose()*S.inverse();
      P = (P.Identity()-K*H)*P;
      st = st + K*y;
      st(0)=remainder(st(0),2*PI);
      if(pos_count>0){
          msg_ = msg;
          publish(msg.header.stamp);
      }
  }
  void PosCallback(const nav_msgs::Odometry& msg){
      pos_count++;
    
      ///
      // st[11] = msg.pose.pose.position.x;
      // st[12] = msg.pose.pose.position.y;
      // st[13] = msg.pose.pose.position.z;
      // return;
      ///
      predict(msg.header.stamp);
      // z = h(st)+v
      // H = dh/dst
      // v ~ N(0,R)
      Matrix<double,_ZPos,_ST> H;
      Matrix<double,_ZPos,1> z;
      z << msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z;
      H.setZero();
      H(0,11)=1;
      H(1,12)=1;
      H(2,13)=1;
      Matrix<double,_ZPos,1> y = z-H*st;
      Matrix<double,_ZPos,_ZPos> S = H*P*H.transpose()+RPos;
      Matrix<double,_ST,_ZPos> K = P*H.transpose()*S.inverse();
      P = (P.Identity()-K*H)*P;
      st = st + K*y;
      st(0)=remainder(st(0),2*PI);
  }
  void GpsCallback(const hellocm_msgs::GPS_Out& msg){
      nav_msgs::Odometry odom;
      odom.header = msg.header;
      odom.header.stamp = msg.time;
      // ROS_WARN("lat long stamp %lf %lf %lf",msg.latitude,msg.longitude,odom.header.stamp);
      vector<double> pos = latlong_to_xy(msg.latitude, msg.longitude);

      // tf::Quaternion q;
      // tf::Quaternion id;
      // id.setRPY(0,0,0);
      // q.setRPY(st[9], st[10], st[0]);
      
      // tf::Transform global_gnss_ = tf::Transform(id, tf::Vector3(pos[0],pos[1],msg.altitude));
      // tf::Transform rot_ = tf::Transform(q, tf::Vector3(0,0,0));
      // tf::Transform odom_gnss_ = tf::Transform(id, tf::Vector3(0,0,1.7-0.5424));

      // tf::Vector3 global_odom_ = (global_gnss_*rot_*odom_gnss_.inverse()).getOrigin();

      // odom.pose.pose.position.x = global_odom_.x();
      // odom.pose.pose.position.y = global_odom_.y();
      odom.pose.pose.position.x = pos[0];
      odom.pose.pose.position.y = pos[1];
      odom.pose.pose.position.z = msg.altitude;
      // ROS_WARN("px py pz %lf %lf %lf",pos[0],pos[1],msg.altitude);
      PosCallback(odom);
  }
  void predict(ros::Time t){
      if(count==0 && pos_count==0){
          t_=t;
          return;
      }
      // st = f(st,u)+w
      // F = df/dst
      // B = df/du
      // w ~ N(0,Q)
      double dt = (t-t_).toSec();
      t_=t;
      Matrix<double,_ST,_ST> F;
      //Matrix<double,_ST,_U> B;
      F.setIdentity();
      for(int i=0; i<3; i++){
          F(i+11,i+1) = dt;
          F(i+11,i+5) = 0.5*sq(dt);
          F(i+1,i+5) = dt;
      }
      F(0,4) = dt;
      F(0,8) = 0.5*sq(dt);
      F(4,8) = dt;
      //B.setZero();
     
      double s_i[3][4] = {{11,12,13,0}, {1,2,3,4}, {5,6,7,8}};
      double timefactors[3] = {trp(dt)/6, sq(dt)/2, dt};
      for(int i=0;i<3;i++){
          for(int j=0;j<3;j++){
              Q(s_i[i][0],s_i[j][0]) = sq(std_accrate)*timefactors[i]*timefactors[j];
              Q(s_i[i][1],s_i[j][1]) = sq(std_accrate)*timefactors[i]*timefactors[j];
              Q(s_i[i][2],s_i[j][2]) = sq(std_acczrate)*timefactors[i]*timefactors[j];
              Q(s_i[i][3],s_i[j][3]) = sq(std_yawaccrate)*timefactors[i]*timefactors[j];
          }
      }
      Q(9,9) = sq(std_rollpitchrate*dt);
      Q(10,10) = sq(std_rollpitchrate*dt);
      st = F*st;
      P = F*P*F.transpose()+Q;
      st(0)=remainder(st(0),2*PI);
  }
  void publish(ros::Time t){
      msg_.header.frame_id = "Fr0";
      msg_.x.clear();
      msg_.v.clear();
      msg_.a.clear();
      for(int i=0; i<3; i++){
          msg_.x.push_back(st(i+11));
          msg_.v.push_back(st(i+1));
          msg_.a.push_back(st(i+5));
      }
      msg_.yaw = st(0);
      msg_.yawrate = st(4);
      msg_.yawacc = st(8);
      msg_.roll = st(9);
      msg_.pitch = st(10);
      pub_.publish(msg_);
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalman_controller");
  ros::NodeHandle n;
   KalmanFusion kf;
  ros::Subscriber sub = n.subscribe("/vehicleinfo",1,&KalmanFusion::Callback,&kf);
  // ros::Subscriber sub_pos = n.subscribe("/Odometry",1,&KalmanFusion::PosCallback,&kf);
  ros::Subscriber sub_gps = n.subscribe("/gps_out",1,&KalmanFusion::GpsCallback,&kf);
  ros::spin();
  return 0;
}
 
 

