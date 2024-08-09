#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

bool update_state_on_clock;
ros::Time last_clock_msg;

ros::Publisher pub_dvl;
float variance;

double RAD_TO_DEG = 180.0 / 3.14159265;

void odom_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    geometry_msgs::PoseWithCovarianceStamped new_msg = *msg;
    new_msg.header.frame_id = "dvl";
    new_msg.pose.covariance[18] = variance;
    new_msg.pose.covariance[24] = variance;
    new_msg.pose.covariance[30] = variance;


    pub_dvl.publish(new_msg);

}


int main(int argc, char **argv) {
    ros::init(argc,argv,"dvl_republish");
    ros::NodeHandle n;

  

    ros::Subscriber odom_sub = n.subscribe("/sensors/dvl/raw",100,&odom_cb);

  

    pub_dvl = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensors/dvl/pose",1);   
    ros::param::get("~variance",variance);


    ros::spin();
    return 0;
}