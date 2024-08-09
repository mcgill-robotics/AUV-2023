#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

bool update_state_on_clock;
ros::Time last_clock_msg;

ros::Publisher pub_pose;
ros::Publisher pub_x;
ros::Publisher pub_y;
ros::Publisher pub_z;
ros::Publisher pub_theta_x;
ros::Publisher pub_theta_y;
ros::Publisher pub_theta_z;
ros::Publisher pub_av;
ros::Publisher pub_imu_status;
ros::Publisher pub_dvl_status;
ros::Publisher pub_depth_status;

double RAD_TO_DEG = 180.0 / 3.14159265;

void broad_cast_pose(const geometry_msgs::Pose& msg);

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) {

    std_msgs::Float64 z;
    z.data = msg->pose.pose.position.z;
    std_msgs::Float64 x;
    x.data = msg->pose.pose.position.x;
    std_msgs::Float64 y;
    y.data = msg->pose.pose.position.y;

    geometry_msgs::Quaternion q_nwu_auv = msg->pose.pose.orientation;
    geometry_msgs::Vector3 av = msg->twist.twist.angular;




    geometry_msgs::Pose pose = msg->pose.pose;

    pub_x.publish(x);
    pub_y.publish(y);
    pub_z.publish(z);
    pub_pose.publish(pose);

    tf2::Quaternion quat_tf2_format(q_nwu_auv.x,q_nwu_auv.y,q_nwu_auv.z,q_nwu_auv.w);
    tf2::Matrix3x3 mat(quat_tf2_format);
    double yaw;
    double pitch;
    double roll;
    mat.getEulerYPR(yaw,pitch,roll);

    std_msgs::Float64 yaw_msg;
    std_msgs::Float64 pitch_msg;
    std_msgs::Float64 roll_msg;

    yaw *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    roll *= RAD_TO_DEG;

    yaw_msg.data = yaw;
    pitch_msg.data = pitch;
    roll_msg.data = roll;

    // ROS_DEBUG("roll: %lf, pitch: %lf, yaw: %lf",roll, pitch, yaw);

    pub_theta_x.publish(roll_msg);
    pub_theta_y.publish(pitch_msg);
    pub_theta_z.publish(yaw_msg);

    broad_cast_pose(pose);

}

void broad_cast_pose(const geometry_msgs::Pose& msg) {
    if(update_state_on_clock) {
        ros::Time now = ros::Time::now();
        if(now == last_clock_msg) {
            return;
        }
        last_clock_msg.sec = now.sec;
        last_clock_msg.nsec = now.nsec;
    }

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped1;
    transformStamped1.header.stamp = ros::Time::now();
    transformStamped1.header.frame_id = "world";
    transformStamped1.child_frame_id = "auv_base";
    transformStamped1.transform.translation.x = msg.position.x;
    transformStamped1.transform.translation.y = msg.position.y;
    transformStamped1.transform.translation.z = msg.position.z;
    transformStamped1.transform.rotation = msg.orientation;
    br.sendTransform(transformStamped1);

    geometry_msgs::TransformStamped transformStamped2;
    transformStamped2.header.stamp = ros::Time::now();
    transformStamped2.header.frame_id = "world_rotation";
    transformStamped2.child_frame_id = "auv_rotation";
    transformStamped2.transform.translation.x = 0;
    transformStamped2.transform.translation.y = 0;
    transformStamped2.transform.translation.z = 0;
    transformStamped2.transform.rotation = msg.orientation;
    br.sendTransform(transformStamped2);
}




int main(int argc, char **argv) {
    ros::init(argc,argv,"odom_republish");
    ros::NodeHandle n;

  

    ros::Subscriber odom_sub = n.subscribe("/odometry/filtered",100,&odom_cb);

  

    pub_pose = n.advertise<geometry_msgs::Pose>("/state/pose",1);
    pub_x = n.advertise<std_msgs::Float64>("/state/x",1);
    pub_y = n.advertise<std_msgs::Float64>("/state/y",1);
    pub_z = n.advertise<std_msgs::Float64>("/state/z",1);
    pub_theta_x = n.advertise<std_msgs::Float64>("/state/theta/x",1);
    pub_theta_y = n.advertise<std_msgs::Float64>("/state/theta/y",1);
    pub_theta_z = n.advertise<std_msgs::Float64>("/state/theta/z",1);
    pub_av = n.advertise<geometry_msgs::Vector3>("/state/angular_velocity",1);


    ros::param::get("/update_state_on_clock", update_state_on_clock);
    



    ros::spin();
    return 0;
}