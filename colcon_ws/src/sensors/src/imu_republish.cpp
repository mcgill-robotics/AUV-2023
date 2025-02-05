#include <sstream>
#include <iostream>

#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <rclcpp/rclcpp.hpp>

/*
    Node that post-processes IMU sensory data and re-publishes it.
*/
class ImuRepublishNode : public rclcpp::Node {
public :
    ImuRepublishNode() : Node("imu_republish") {
        this->declare_parameter<float>("angular_velocity_variance", 0.0);
        this->declare_parameter<float>("acceleration_variance", 0.0);

        this->get_parameter("angular_velocity_variance", angular_velocity_variance);
        this->get_parameter("acceleration_variance", acceleration_variance);

        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/sensors/imu/data", 1);
        odom_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/sensors/imu/raw", 100, std::bind(&ImuRepublishNode::odom_cb, this, std::placeholders::_1)
        );
    }

private:
    void odom_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
        auto new_msg = *msg;
        new_msg.header.frame_id = "imu";

        new_msg.angular_velocity_covariance[0] = angular_velocity_variance;
        new_msg.angular_velocity_covariance[4] = angular_velocity_variance;
        new_msg.angular_velocity_covariance[8] = angular_velocity_variance;
        
        new_msg.linear_acceleration_covariance[0] = acceleration_variance;
        new_msg.linear_acceleration_covariance[4] = acceleration_variance;
        new_msg.linear_acceleration_covariance[8] = 10;

        if(new_msg.orientation.w < 0) {
            new_msg.orientation.w *= -1;
            new_msg.orientation.x *= -1;
            new_msg.orientation.y *= -1;
            new_msg.orientation.z *= -1;
        }
        pub_imu->publish(new_msg);

    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr odom_sub;
    float angular_velocity_variance;
    float acceleration_variance;

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImuRepublishNode>();
    rclcpp::spin(node);

    return 0;
}