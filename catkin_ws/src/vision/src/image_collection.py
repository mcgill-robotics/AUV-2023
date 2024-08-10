#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
import os

from sensor_msgs.msg import Image


def front_cam_image_callback(msg):
    global front_cam_cur_image
    front_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def down_cam_image_callback(msg):
    global down_cam_cur_image
    down_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def save_image(output_dir, is_front_cam):
    if is_front_cam:
        cur_image = front_cam_cur_image
    else:
        cur_image = down_cam_cur_image

    filename = os.path.join(output_dir, "image_{}.jpg".format(rospy.Time.now()))
    cv2.imwrite(filename, cur_image)
    print(f'{"Front cam" if is_front_cam else "Down cam"} image saved')


if __name__ == "__main__":
    rospy.init_node("image_collection")

    bridge = CvBridge()
    FRONT_CAM_DATA_DIR = rospy.get_param("~image_data_dir/front_cam")
    DOWN_CAM_DATA_DIR = rospy.get_param("~image_data_dir/down_cam")
    if not os.path.exists(FRONT_CAM_DATA_DIR):
        os.makedirs(FRONT_CAM_DATA_DIR)
    if not os.path.exists(DOWN_CAM_DATA_DIR):
        os.makedirs(DOWN_CAM_DATA_DIR)

    front_cam_cur_image, down_cam_cur_image = None, None

    front_cam_image_sub = rospy.Subscriber(
        "/vision/front_cam/color/image_raw", Image, front_cam_image_callback
    )
    down_cam_image_sub = rospy.Subscriber(
        "/vision/down_cam/image_raw", Image, down_cam_image_callback
    )

    print(
        "These are the following options:\n- To take a screen shot using front cam, press [z]\n- To take a screen shot using down cam, press [x]"
    )
    while not rospy.is_shutdown():
        usr_choice = input("Select an option: ")
        if usr_choice == "z":
            save_image(FRONT_CAM_DATA_DIR, is_front_cam=True)
        elif usr_choice == "x":
            save_image(DOWN_CAM_DATA_DIR, is_front_cam=False)
        else:
            print("Not a valid option!!!")