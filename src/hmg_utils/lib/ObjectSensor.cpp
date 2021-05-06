#include "ObjectSensor.h"

#include "std_msgs/Float32.h"



ObjectSensor::ObjectSensor(){ 
}

ObjectSensor::ObjectSensor(double x, double y, double z, double roll, double pitch, double yaw){
    this->x = x;
    this->y = y;
    this->z = z;
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;

    id.setRPY(0,0,0);
    origin = tf::Vector3(0,0,0);

    tf::Quaternion q;
    tf::Vector3 v = tf::Vector3(x,y,z);
    q.setRPY(roll, pitch, yaw);
    sensor_tf = tf::Transform(q,v);

    ROS_WARN("COMPLETE TO INITIALIZE OBJECT SENSOR!!");
}

void ObjectSensor::setBoardCaster(ros::Subscriber* subptr, ros::Publisher* pubptr){
    pub = pubptr;
    sub = subptr;
}

void ObjectSensor::callback(const hellocm_msgs::ObjectSensor::ConstPtr& msg){
    ROS_INFO("?????");
    vehicle_detection::tracker_output rt;
    data = *msg;
    
    rt.header = msg->header;
    double t =msg->header.stamp.toSec();


    vector<vehicle_detection::tracking_object> objects;


    for(int i=0;i<msg->Objects.size(); i++){
        hellocm_msgs::ObjectSensorObj cur = msg->Objects[i];
        vehicle_detection::tracking_object object;
        
        /////////////////////////////////
        object.obj_num = stoi(cur.Name);
        //object.obj_num = stoi(cur.Name.substr(1));
        ////////////////////////////
        object.obj_class = cur.type;
        object.w = cur.w;
        object.h = cur.l;

        tf::Transform T = odom * sensor_tf;


        /* time */
        //object.position.t = t;

        /* calculate position */
        tf::Vector3 ref_position = tf::Vector3(cur.RefPnt.ds[0],cur.RefPnt.ds[1],cur.RefPnt.ds[2]);
        tf::Quaternion ref_orientation;
        ref_orientation.setRPY(cur.RefPnt.r_zyx[0],cur.RefPnt.r_zyx[1],cur.RefPnt.r_zyx[2]);
        tf::Transform ref_T = T * tf::Transform(ref_orientation, ref_position);
        tf::Vector3 obj_position = tf::Vector3(cur.l/2,0,0);
        tf::Transform obj_T = ref_T * tf::Transform(id, obj_position);
        tf::Vector3 pos = obj_T.getOrigin();
        double r,p,y;
        obj_T.getBasis().getRPY(r,p,y);

        object.x = pos.x();
        object.y = pos.y();
        object.yaw = y;

        /* calculate velocity */
        tf::Vector3 velocity_position = tf::Vector3(cur.RefPnt.dv[0],cur.RefPnt.dv[1],cur.RefPnt.dv[2]);
        velocity_position -= (ref_position+sensor_tf.getOrigin()).cross(vehicle_ang);
        tf::Transform vel_T = tf::Transform(odom.getRotation(),origin) * tf::Transform(id, velocity_position);
        tf::Vector3 vel = vel_T.getOrigin();


        object.x_d = vehicle_vel.x() + vel.x();
        object.y_d = vehicle_vel.y() + vel.y();
        //object.yaw = atan2(object.y_d, object.x_d);

        objects.push_back(object);
    }
    rt.data = objects;
    

    pub->publish(rt);
    
}

void ObjectSensor::update(tf::Transform odom, tf::Vector3 vel, tf::Vector3 ang){
    this->odom = odom;
    vehicle_vel = vel;
    vehicle_ang = ang;
}