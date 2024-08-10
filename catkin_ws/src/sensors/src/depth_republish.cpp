#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


bool update_state_on_clock;
ros::Time last_clock_msg;

ros::Publisher pub_pose;
float variance;

double RAD_TO_DEG = 180.0 / 3.14159265;

void broad_cast_pose(const geometry_msgs::Pose& msg);

void odom_cb(const std_msgs::Float64::ConstPtr& msg) {

    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.pose.pose.position.z = msg->data *-1;
    pose_msg.pose.covariance[2*6 + 2] = variance; //double check this later
    pose_msg.header.frame_id = "depth";
    pose_msg.header.stamp = ros::Time::now();

    pub_pose.publish(pose_msg);

}


int main(int argc, char **argv) {
    ros::init(argc,argv,"depth_republish");
    ros::NodeHandle n;

  

    ros::Subscriber odom_sub = n.subscribe("/sensors/depth/z",100,&odom_cb);

  

    pub_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensors/depth/pose",1);   
    ros::param::get("~variance",variance);


    ros::spin();
    return 0;
}