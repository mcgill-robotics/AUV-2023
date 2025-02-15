#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from sensor_msgs import point_cloud2

def rbg_callback(msg):
    global rgb
    temp = bridge.imgmsg_to_cv2(msg)
    rgb = temp / 255


def depth_callback(msg):
    global depth
    temp = bridge.imgmsg_to_cv2(msg)
    depth = temp / DEPTH_SCALE_FACTOR


def camera_info_callback(msg):
    global fx, fy, cx, cy, width, height, x_over_z_map, y_over_z_map
    # if(y_over_z_map is not None): return
    fx = msg.K[0]
    fy = msg.K[4]
    cx = msg.K[2]
    cy = msg.K[5]

    width = msg.width
    height = msg.height

    u_map = np.tile(np.arange(width), (height, 1)) + 1
    v_map = np.tile(np.arange(height), (width, 1)).T + 1

    x_over_z_map = (cx - u_map) / fx
    y_over_z_map = (cy - v_map) / fy


def convert_from_uvd(width, height):
    if y_over_z_map is not None:
        time = rospy.Time(0)
        xyz_rgb_image = get_xyz_rgb_image(
            rgb, depth, width, height, x_over_z_map, y_over_z_map
        )

        xyz_rgb_image = xyz_rgb_image.reshape((width * height, 6))
        xyz_rgb_image = xyz_rgb_image.astype(np.float32)
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("r", 12, PointField.FLOAT32, 1),
            PointField("g", 16, PointField.FLOAT32, 1),
            PointField("b", 20, PointField.FLOAT32, 1),
        ]

        header = Header()
        header.stamp = time
        header.frame_id = "auv_base"
        pub_msg = point_cloud2.create_cloud(
            header=header, fields=fields, points=xyz_rgb_image
        )
        return pub_msg


def get_point_cloud_image(
    bridge, color, z_map, width, height, x_over_z_map, y_over_z_map
):
    if y_over_z_map is not None:
        xyz_rgb_image = get_xyz_rgb_image(
            color, z_map, width, height, x_over_z_map, y_over_z_map
        )
        point_cloud_image = bridge.cv2_to_imgmsg(np.float32(xyz_rgb_image[:, :, :3]))
        return point_cloud_image


def get_xyz_rgb_image(color, z_map, width, height, x_over_z_map, y_over_z_map):
    if y_over_z_map is not None:
        xyz_rgb_image = np.zeros((height, width, 6))
        xyz_rgb_image[:, :, 3:6] = color[:, :, 0:3]

        x_map = x_over_z_map * z_map
        y_map = y_over_z_map * z_map

        xyz_rgb_image[:, :, 0] = z_map + rospy.get_param("front_cam_x_offset", 0)
        xyz_rgb_image[:, :, 1] = x_map + rospy.get_param("front_cam_y_offset", 0)
        xyz_rgb_image[:, :, 2] = y_map + rospy.get_param("front_cam_z_offset", 0)

        return xyz_rgb_image


def get_xyz_image(z_map, width, height, x_over_z_map, y_over_z_map):
    if y_over_z_map is not None:
        xyz_image = np.zeros((height, width, 3))

        x_map = x_over_z_map * z_map
        y_map = y_over_z_map * z_map

        xyz_image[:, :, 0] = z_map + rospy.get_param("front_cam_x_offset", 0)
        xyz_image[:, :, 1] = x_map + rospy.get_param("front_cam_y_offset", 0)
        xyz_image[:, :, 2] = y_map + rospy.get_param("front_cam_z_offset", 0)

        return xyz_image


if __name__ == "__main__":
    rospy.init_node("point_cloud_sim")

    bridge = CvBridge()

    fx = None
    fy = None
    cx = None
    cy = None
    width = None
    height = None

    x_over_z_map = None
    y_over_z_map = None
    convert_map = None
    rgb = None
    depth = None

    DEPTH_SCALE_FACTOR = rospy.get_param("depth_map_scale_factor")

    point_cloud_pub = rospy.Publisher(
        "vision/front_cam/point_cloud_raw", PointCloud2, queue_size=3
    )

    camera_info_sub = rospy.Subscriber(
        "/vision/front_cam/camera_info", CameraInfo, camera_info_callback
    )
    depth_sub = rospy.Subscriber(
        "/vision/front_cam/aligned_depth_to_color/image_raw", Image, depth_callback
    )
    rgb_sub = rospy.Subscriber("/vision/front_cam/color/image_raw", Image, rbg_callback)
    
    # aligned_imaged_sub = rospy.Subscriber('/vision/front_cam/aligned_depth_to_color/image_raw', Image, algined_cb)

    while not rospy.is_shutdown():
        if rgb is not None and depth is not None:
            msg = convert_from_uvd(width, height)
            if msg is not None:
                point_cloud_pub.publish(msg)