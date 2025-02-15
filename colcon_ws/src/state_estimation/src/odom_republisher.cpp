#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class OdomRepublishNode : public rclcpp::Node
{
public:
    OdomRepublishNode()
        : Node("odom_republish")
    {
        // Declare and get parameters
        this->declare_parameter("update_state_on_clock", false);
        this->get_parameter("update_state_on_clock", update_state_on_clock);

        // Initialize publishers
        pub_pose = this->create_publisher<geometry_msgs::msg::Pose>("/state/pose", 10);
        pub_x = this->create_publisher<std_msgs::msg::Float64>("/state/x", 10);
        pub_y = this->create_publisher<std_msgs::msg::Float64>("/state/y", 10);
        pub_z = this->create_publisher<std_msgs::msg::Float64>("/state/z", 10);
        pub_theta_x = this->create_publisher<std_msgs::msg::Float64>("/state/theta/x", 10);
        pub_theta_y = this->create_publisher<std_msgs::msg::Float64>("/state/theta/y", 10);
        pub_theta_z = this->create_publisher<std_msgs::msg::Float64>("/state/theta/z", 10);
        pub_av = this->create_publisher<geometry_msgs::msg::Vector3>("/state/angular_velocity", 10);

        // Initialize subscribers
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered",
            10,
            std::bind(&OdomRepublishNode::odom_cb, this, std::placeholders::_1)
        );
        depth_sub = this->create_subscription<std_msgs::msg::Float64>(
            "/sensors/depth/z",
            10,
            std::bind(&OdomRepublishNode::depth_cb, this, std::placeholders::_1)
        );

        // Initialize transform broadcaster
        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void depth_cb(const std_msgs::msg::Float64::SharedPtr msg)
    {
        depth = msg->data * -1;
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto z = std_msgs::msg::Float64();
        z.data = depth;
        auto x = std_msgs::msg::Float64();
        x.data = msg->pose.pose.position.x;
        auto y = std_msgs::msg::Float64();
        y.data = msg->pose.pose.position.y;

        auto q_nwu_auv = msg->pose.pose.orientation;
        auto av = msg->twist.twist.angular;

        auto pose = msg->pose.pose;
        pose.position.z = depth;

        pub_x->publish(x);
        pub_y->publish(y);
        pub_z->publish(z);
        pub_pose->publish(pose);

        tf2::Quaternion quat_tf2_format(q_nwu_auv.x, q_nwu_auv.y, q_nwu_auv.z, q_nwu_auv.w);
        tf2::Matrix3x3 mat(quat_tf2_format);
        double yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);

        std_msgs::msg::Float64 yaw_msg;
        std_msgs::msg::Float64 pitch_msg;
        std_msgs::msg::Float64 roll_msg;

        yaw *= RAD_TO_DEG;
        pitch *= RAD_TO_DEG;
        roll *= RAD_TO_DEG;

        yaw_msg.data = yaw;
        pitch_msg.data = pitch;
        roll_msg.data = roll;

        pub_theta_x->publish(roll_msg);
        pub_theta_y->publish(pitch_msg);
        pub_theta_z->publish(yaw_msg);

        broad_cast_pose(pose);
    }

    void broad_cast_pose(const geometry_msgs::msg::Pose& msg)
    {
        if (update_state_on_clock)
        {
            auto now = this->get_clock()->now();
            if (now == last_clock_msg)
            {
                return;
            }
            last_clock_msg = now;
        }

        geometry_msgs::msg::TransformStamped transformStamped1;
        transformStamped1.header.stamp = this->get_clock()->now();
        transformStamped1.header.frame_id = "world";
        transformStamped1.child_frame_id = "auv_base";
        transformStamped1.transform.translation.x = msg.position.x;
        transformStamped1.transform.translation.y = msg.position.y;
        transformStamped1.transform.translation.z = msg.position.z;
        transformStamped1.transform.rotation = msg.orientation;
        br->sendTransform(transformStamped1);

        geometry_msgs::msg::TransformStamped transformStamped2;
        transformStamped2.header.stamp = this->get_clock()->now();
        transformStamped2.header.frame_id = "world_rotation";
        transformStamped2.child_frame_id = "auv_rotation";
        transformStamped2.transform.translation.x = 0;
        transformStamped2.transform.translation.y = 0;
        transformStamped2.transform.translation.z = 0;
        transformStamped2.transform.rotation = msg.orientation;
        br->sendTransform(transformStamped2);
    }

    // Members
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_pose;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_x;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_y;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_z;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_theta_x;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_theta_y;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_theta_z;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr pub_av;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr depth_sub;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    bool update_state_on_clock;
    rclcpp::Time last_clock_msg;

    double depth;
    double RAD_TO_DEG = 180.0 / 3.14159265;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomRepublishNode>());
    rclcpp::shutdown();
    return 0;
}
