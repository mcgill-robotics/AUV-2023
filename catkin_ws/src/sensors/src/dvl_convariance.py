#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from tf import transformations
import numpy as np

RAD_PER_DEG = np.pi / 180.0


def parse_dead_reckon_report(line, quat_variance):
    tokens = line.split(",")
    time_stamp = float(tokens[1])
    x = float(tokens[2])
    y = float(tokens[3])
    z = float(tokens[4])
    std = float(tokens[5])
    roll = float(tokens[6])
    pitch = float(tokens[7])
    yaw = float(tokens[8])
    status = bool(tokens[9])
    return [roll, pitch, yaw]


def pose_callback(msg):
    """Callback for PoseWithCovarianceStamped subscriber."""
    rospy.loginfo("Received pose:")
    rospy.loginfo(f"Position: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, z={msg.pose.pose.position.z}")
    rospy.loginfo(f"Orientation: x={msg.pose.pose.orientation.x}, y={msg.pose.pose.orientation.y}, z={msg.pose.pose.orientation.z}, w={msg.pose.pose.orientation.w}")


def twist_callback(msg):
    """Callback for TwistWithCovarianceStamped subscriber."""
    rospy.loginfo("Received twist:")
    rospy.loginfo(f"Linear Velocities: x={msg.twist.twist.linear.x}, y={msg.twist.twist.linear.y}, z={msg.twist.twist.linear.z}")
    rospy.loginfo(f"Angular Velocities: x={msg.twist.twist.angular.x}, y={msg.twist.twist.angular.y}, z={msg.twist.twist.angular.z}")


def main():
    rospy.init_node("waterlinked_driver")

    # Publishers
    pub_vr = rospy.Publisher("/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=1)
    pub_dr = rospy.Publisher("/sensors/dvl/pose", PoseWithCovarianceStamped, queue_size=1)

    # Subscribers
    rospy.Subscriber("/sensors/dvl/pose", PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber("/sensors/dvl/twist", TwistWithCovarianceStamped, twist_callback)

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")
    quat_variance = rospy.get_param("~quat_variance")

    try:
        conn = serial.Serial(port)
    except serial.serialutil.SerialException:
        rospy.logerr("ERR: /dev/dvl directory does not exist")
        rospy.sleep(5)
        exit()

    conn.timeout = 10
    conn.baudrate = baudrate

    if not conn.isOpen():
        conn.open()

    conn.send_break()
    conn.flush()

    conn.write("wcr\r\n".encode("utf-8"))
    conn.flush()

    rospy.loginfo("Resetting dead reckoning.")

    while conn.is_open and not rospy.is_shutdown():
        try:
            line = conn.readline().decode("utf-8")
            if line.startswith("wra"):
                rospy.loginfo("INFO: DVL dead reckoning reset successful.")
                break
            elif line.startswith("wrn"):
                rospy.logwarn("WARN: DVL dead reckoning reset failed.")
                break
        except Exception as e:
            rospy.logerr(e)
            break

    start = rospy.Time.now()
    eulers = []
    while conn.is_open and not rospy.is_shutdown() and rospy.Time.now() - start < rospy.Duration(30):
        try:
            line = conn.readline().decode("utf-8")
            if line.startswith("wrp"):
                eulers.append(parse_dead_reckon_report(line, quat_variance))
        except Exception as e:
            rospy.logerr(e)
            conn.close()
            exit()

    eulers = np.array(eulers)
    eulers_cov = np.cov(eulers.T)
    rospy.loginfo("Eulers Covariance:")
    rospy.loginfo(eulers_cov)

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
