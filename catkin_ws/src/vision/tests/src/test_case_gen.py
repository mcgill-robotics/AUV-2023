#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from time import strftime
import math
import cv2
import os
from auv_msgs.msg import VisionObjectArray, VisionObject


def pose_callback(msg):
    global pose
    pose = msg


def down_callback(msg):
    global down_cam
    data = bridge.imgmsg_to_cv2(msg, "bgr8")
    down_cam = data


def front_callback(msg):
    global front_cam
    data = bridge.imgmsg_to_cv2(msg, "bgr8")
    front_cam = data


def depth_image_callback(msg):
    global depth_image
    data = bridge.imgmsg_to_cv2(msg)
    depth_image = data


def save_data():
    global pose, front_cam, down_cam, depth_image
    if (
        pose is not None
        and front_cam is not None
        and down_cam is not None
        and depth_image is not None
    ):
        title = strftime("%d_%m_%Y_%H:%M:%S")
        output_dir = test_dir + "/" + title
        os.mkdir(output_dir)
        text = open(output_dir + "/pose.txt", "w")
        text.write(
            f"{pose.position.x},{pose.position.y},{pose.position.z},{pose.orientation.w},{pose.orientation.x},{pose.orientation.y},{pose.orientation.z}"
        )
        cv2.imwrite(output_dir + "/front_cam.png", front_cam)
        cv2.imwrite(output_dir + "/down_cam.png", down_cam)
        cv2.imwrite(output_dir + "/depth.png", depth_image)
        print("captured")
    else:
        print("missing some info")


if __name__ == "__main__":
    rospy.init_node("test_case_gen")

    pose = None
    front_cam = None
    down_cam = None
    bridge = CvBridge()
    test_dir = rospy.get_param("~test_dir")
    pose_sub = rospy.Subscriber("/state/pose", Pose, pose_callback)
    image_sub1 = rospy.Subscriber("/vision/down_cam/image_raw", Image, down_callback)
    image_sub2 = rospy.Subscriber(
        "/vision/front_cam/color/image_raw", Image, front_callback
    )
    image_sub3 = rospy.Subscriber(
        "/vision/front_cam/aligned_depth_to_color/image_raw",
        Image,
        depth_image_callback,
    )
    while input("press enter to capture, x to escape") != "x":
        print("trying to capture")
        save_data()
