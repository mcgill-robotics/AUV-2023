#!/usr/bin/env python3

import rospy
import numpy as np
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge

from common_utils import crop_to_bbox
from point_cloud import get_xyz_image

from auv_msgs.msg import VisionObjectArray  # Not Used
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo



class VisionState:
    def __init__(self):
        self.is_paused = False
        # Not paused.
        self.position = None
        self.q_auv = None
        self.theta_z = None
        self.point_cloud = None
        # Paused.
        self.position_while_paused = None
        self.theta_z_while_paused = None
        self.q_auv_while_paused = None
        self.point_cloud_while_paused = None
        
        self.bgr_image = None
        self.depth = None
        self.width = None
        self.height = None
        self.x_over_z_map = None
        self.y_over_z_map = None

        self.bridge = CvBridge()

        self.DEPTH_SCALE_FACTOR = rospy.get_param("depth_map_scale_factor")

        self.eps = rospy.get_param(
            "max_distance_for_point_cloud_fill_cleaning"
        )  

        self.pose_sub = rospy.Subscriber("/state/pose", Pose, self.update_pose)
        self.theta_z_sub = rospy.Subscriber(
            "/state/theta/z", Float64, self.update_theta_z
        )
        # Update the point cloud whenever the current image is updated.
        self.camera_info_sub = rospy.Subscriber(
            "/zed/zed_node/depth/camera_info", CameraInfo, self.update_camera_info
        )
        self.depth_sub = rospy.Subscriber(
            "/zed/zed_node/depth/depth_registered",
            Image,
            self.update_depth,
        )

    def update_theta_z(self, msg):
        if self.is_paused:
            self.theta_z_while_paused = float(msg.data)
        else:
            self.theta_z = float(msg.data)

    def update_pose(self, msg):
        if self.is_paused:
            self.position_while_paused = msg.position
            self.q_auv_while_paused = np.quaternion(
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            )
        else:
            self.position = msg.position
            self.q_auv = np.quaternion(
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
            )

    def update_point_cloud(self):
        if self.is_paused:
            self.point_cloud_while_paused = np.copy(
                get_xyz_image(
                    self.depth,
                    self.width,
                    self.height,
                    self.x_over_z_map,
                    self.y_over_z_map,
                )
            )
        else:
            self.point_cloud = np.copy(
                get_xyz_image(
                    self.depth,
                    self.width,
                    self.height,
                    self.x_over_z_map,
                    self.y_over_z_map,
                )
            )

    def clean_point_cloud(self, point_cloud, bgr):
        # Find the closest point to the camera.
        initial_point_cloud_shape = point_cloud.shape
        point_cloud = point_cloud.reshape(-1, 3)
        point_cloud[
            point_cloud[:, 0]
            < rospy.get_param("min_distance_for_valid_point_cloud_point")
        ] = 10000  # Ignore depth values which are less than 0.5m.
        closest_point_index = np.argmin(point_cloud[:, 0])

        # Maximum distance between two samples for them to be 
        # considered as in the same neighborhood.
        
        min_samples = 10  # The number of samples in a neighborhood for a point to be considered as a core point

        # Perform DBSCAN clustering
        dbscan = DBSCAN(eps=self.eps, min_samples=min_samples)
        labels = dbscan.fit_predict(point_cloud)

        # Extract the cluster containing the seed point
        closest_object_label = labels[closest_point_index]
        object_mask = labels == closest_object_label
        point_cloud[~object_mask] = np.array([np.nan] * 3)

        point_cloud = point_cloud.reshape(initial_point_cloud_shape)
        object_mask = object_mask.reshape(initial_point_cloud_shape[0:2])

        if rospy.get_param("debug_point_cloud_cleaning"):
            debug_image = np.uint8(np.zeros((point_cloud.shape)))
            debug_image[object_mask] = np.array([0, 0, 255])
            rospy.Publisher(
                "/vision/debug/point_cloud_clean", Image, queue_size=0
            ).publish(self.bridge.cv2_to_imgmsg(debug_image, "bgr8"))

        return point_cloud

    def get_point_cloud(self, bbox=None):
        if bbox is None:
            # bbox is bounding box: surrounds bounds an object or a specific area of interest in a robot's perception system
            return self.clean_point_cloud(
                np.copy(self.point_cloud), np.copy(self.bgr_image)
            )
        else:
            return self.clean_point_cloud(
                crop_to_bbox(self.point_cloud, bbox, copy=True),
                crop_to_bbox(self.bgr_image, bbox, copy=True),
            )

    def update_depth(self, msg):
        temp = self.bridge.imgmsg_to_cv2(msg)
        self.depth = temp / self.DEPTH_SCALE_FACTOR
        self.update_point_cloud()

    def update_camera_info(self, msg):
        fx = msg.K[0]
        fy = msg.K[4]
        cx = msg.K[2]
        cy = msg.K[5]

        self.width = msg.width
        self.height = msg.height

        u_map = np.tile(np.arange(self.width), (self.height, 1)) + 1
        v_map = np.tile(np.arange(self.height), (self.width, 1)).T + 1

        self.x_over_z_map = (cx - u_map) / fx
        self.y_over_z_map = (cy - v_map) / fy
        if self.depth is not None:
            self.update_point_cloud()

    def pause(self):
        self.is_paused = True

    def resume(self):
        if self.position_while_paused is not None:
            self.position = self.position_while_paused
        if self.theta_z_while_paused is not None:
            self.theta_z = self.theta_z_while_paused
        if self.q_auv_while_paused is not None:
            self.q_auv = self.q_auv_while_paused
        if self.point_cloud_while_paused is not None:
            self.point_cloud = self.point_cloud_while_paused

        self.position_while_paused = None
        self.theta_z_while_paused = None
        self.q_auv_while_paused = None
        self.point_cloud_while_paused = None

        self.is_paused = False