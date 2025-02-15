#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from time import strftime
import cv2
from pyproj import Transformer
from pyproj import CRS
from pyproj.aoi import AreaOfInterest
from pyproj.database import query_utm_crs_info
import time
import os
import json
import numpy as np
import quaternion
from tf import transformations

DEG_PER_RAD = 180 / np.pi


def camera_info_callback(msg):
    global output_dir, title, camera_info_seen

    if camera_info_seen:
        return
    camera_info_seen = True

    # Extract camera parameters
    width = msg.width
    height = msg.height
    distortion_model = msg.distortion_model
    focal_x = msg.K[0]
    focal_y = msg.K[4]
    c_x = msg.K[2]
    c_y = msg.K[5]
    k1 = msg.D[0]
    k2 = msg.D[1]
    p1 = msg.D[2]
    p2 = msg.D[3]
    k3 = msg.D[4]

    # Format camera parameters
    camera_params = {
        f"{width} {height} {distortion_model} {focal_x:.8f}": {
            "projection_type": distortion_model,
            "width": width,
            "height": height,
            "focal_x": focal_x,
            "focal_y": focal_y,
            "c_x": (c_x - (width / 2)) / (width / 2),
            "c_y": (c_y - (height / 2)) / (height / 2),
            "k1": k1,
            "k2": k2,
            "p1": p1,
            "p2": p2,
            "k3": k3,
        }
    }

    camera_txt = open(output_dir + f"/cameras.json", "w")
    camera_txt.write(json.dumps(camera_params))
    camera_txt.close()


def pose_callback(msg):
    global gps, depth, seen_pose, backwards, north_offset, east_offset, roll, pitch, yaw
    new_north, new_east = north_offset + msg.position.x, east_offset - msg.position.y
    gps = backwards.transform(new_east, new_north)
    depth = msg.position.z

    quaternion_nwu = np.quaternion(
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
    )
    quaternion_ned = (
        np.quaternion(0, 1, 0, 0) * quaternion_nwu * np.quaternion(0, 1, 0, 0)
    )
    yaw, pitch, roll = transformations.euler_from_quaternion(
        [quaternion_ned.x, quaternion_ned.y, quaternion_ned.z, quaternion_ned.w], "szyx"
    )
    yaw *= DEG_PER_RAD
    pitch *= DEG_PER_RAD
    roll *= DEG_PER_RAD
    seen_pose = True


def image_callback(msg):
    global image, seen_image
    data = bridge.imgmsg_to_cv2(msg, "bgr8")
    image = data
    seen_image = True


def init_text_file():
    global output_txt, output_dir, title
    output_txt = open(output_dir + f"/geo.txt", "w")
    output_txt.write("EPSG:4326\n")


def save_data():
    global gps, depth, image, output_txt, video, roll, pitch, yaw, seen_pose
    print(gps, seen_image)
    if seen_pose and seen_image:
        print("saving")
        millis = str(int(round(time.time() * 1000)))[0:3]
        the_time = strftime(f"%H:%M:%S.{millis}")
        title = the_time + ".jpg"
        output_txt.write(
            f"{title} {gps[0]:10.13f} {gps[1]:10.13f} {depth} {yaw} {pitch} {roll}\n"
        )
        cv2.imwrite(output_dir + "/" + title, image)


def shutdown():
    global output_txt
    print("shutting down")
    output_txt.close()


if __name__ == "__main__":
    rospy.init_node("data_collection")

    gps = None
    depth = None
    seen_pose = False
    seen_image = False
    image = None
    video = None
    roll = None
    pitch = None
    yaw = None
    title = strftime("%d_%m_%Y_%H:%M:%S")
    bridge = CvBridge()
    laditude_offset = rospy.get_param("~laditude_offset")
    longitude_offset = rospy.get_param("~longitude_offset")
    frame_rate = rospy.get_param("~frame_rate")
    data_dir = rospy.get_param("~output_dir")
    camera_info_seen = False

    output_dir = f"{data_dir}/{title}"
    os.mkdir(output_dir)

    utm_crs_list = query_utm_crs_info(
        datum_name="WGS 84",
        area_of_interest=AreaOfInterest(
            west_lon_degree=longitude_offset,
            south_lat_degree=laditude_offset,
            east_lon_degree=longitude_offset,
            north_lat_degree=laditude_offset,
        ),
    )
    utm_crs = CRS.from_epsg(utm_crs_list[0].code)
    forwards = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)
    backwards = Transformer.from_crs(utm_crs, "EPSG:4326", always_xy=True)
    east_offset, north_offset = forwards.transform(laditude_offset, longitude_offset)

    pose_sub = rospy.Subscriber("/state/pose", Pose, pose_callback)
    image_sub = rospy.Subscriber("/vision/down_cam/image_raw", Image, image_callback)
    camera_info_sub = rospy.Subscriber(
        "/vision/down_cam/camera_info", CameraInfo, camera_info_callback
    )

    init_text_file()
    while input("press any button to capture, x to finish") != "x":
        save_data()
    shutdown()
