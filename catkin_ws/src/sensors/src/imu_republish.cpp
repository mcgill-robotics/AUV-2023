#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <sensor_msgs/Imu.h>

bool update_state_on_clock;
ros::Time last_clock_msg;

ros::Publisher pub_imu;
float angular_velocity_variance;
float acceleration_variance;

double RAD_TO_DEG = 180.0 / 3.14159265;

void odom_cb(const sensor_msgs::Imu::ConstPtr& msg) {

    sensor_msgs::Imu new_msg = *msg;
    new_msg.header.frame_id = "imu";
    new_msg.angular_velocity_covariance[0] = angular_velocity_variance;
    new_msg.angular_velocity_covariance[4] = angular_velocity_variance;
    new_msg.angular_velocity_covariance[8] = angular_velocity_variance;

    new_msg.linear_acceleration_covariance[0] = acceleration_variance;
    new_msg.linear_acceleration_covariance[4] = acceleration_variance;
    new_msg.linear_acceleration_covariance[8] = acceleration_variance * 10;

    if(new_msg.orientation.w < 0) {
        new_msg.orientation.w *= -1;
        new_msg.orientation.x *= -1;
        new_msg.orientation.y *= -1;
        new_msg.orientation.z *= -1;
    }

    pub_imu.publish(new_msg);

}


int main(int argc, char **argv) {
    ros::init(argc,argv,"imu_republish");
    ros::NodeHandle n;

  

    ros::Subscriber odom_sub = n.subscribe("/sensors/imu/raw",100,&odom_cb);

  

    pub_imu = n.advertise<sensor_msgs::Imu>("/sensors/imu/data",1);   
    ros::param::get("~angular_velocity_variance",angular_velocity_variance);
    ros::param::get("~acceleration_variance",acceleration_variance);


    ros::spin();
    return 0;
}