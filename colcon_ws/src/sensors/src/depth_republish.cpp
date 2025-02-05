
#include <functional>
#include <sstream>
#include <iostream>

#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

class DepthRepublishNode : public rclcpp::Node {
public:
    DepthRepublishNode() : Node("depth_republish") {
        this->declare_parameter<float>("variance", 0.0);
        this->get_parameter("~variance", variance);

        odom_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/sensors/depth/z", 100, std::bind(&DepthRepublishNode::odom_cb, this, std::placeholders::_1)
        );
        
        pub_pose = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/sensors/depth/pose", 1
        );
    }

private:
    void odom_cb(const std_msgs::msg::Float64::SharedPtr msg) {
        auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
        pose_msg.pose.pose.position.z = msg->data * -1;
        pose_msg.pose.covariance[2*6 + 2] = variance; //Covariance matrix element for z - prev comment said double check this later
        pose_msg.header.frame_id = "depth";
        pose_msg.header.stamp = this->get_clock()->now();

        pub_pose->publish(pose_msg);
        }

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr odom_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose;
    float variance;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DepthRepublishNode>();
    rclcpp::spin(node);
    
    return 0;
}
