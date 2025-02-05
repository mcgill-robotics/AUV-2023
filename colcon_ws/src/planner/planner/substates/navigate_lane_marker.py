#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock
from rclpy import Duration
import smach
from .utility.functions import *
import threading
from std_msgs.msg import String


class NavigateLaneMarker(smach.State):
    def __init__(self, control, mapping, state, origin_class, node):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.node = node

        self.origin_class = origin_class
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = self.node.get_parameter("navigate_lane_marker_time_limit").get_parameter_value()
        self.centering_dist_threshold = self.node.get_parameter("center_dist_threshold").get_parameter_value()
        self.centering_delta_increment = self.node.get_parameter("centering_delta_increment").get_parameter_value()

        self.pub_mission_display = self.node.create_publisher(
            String, "/mission_display", 1
        )
    
    def timer_thread_func(self):
        self.pub_mission_display.publish("Lane Marker Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, ud):
        self.node.get_logger().info("Starting lane marker navigation.")
        self.pub_mission_display.publish("Lane")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, self.node.get_parameter("down_cam_search_depth").get_parameter_value()))
        self.control.flatten()

        if self.origin_class == "":  # use 0,0 position as origin
            origin_position = (0, 0)
        else:
            origin_obj = self.mapping.getClosestObject(
                cls=self.origin_class, pos=(self.state.x, self.state.y)
            )
            origin_position = (origin_obj[1], origin_obj[2])

        lane_marker_obj = self.mapping.getClosestObject(
            cls="Lane Marker", pos=(origin_position[0], origin_position[1])
        )
        if lane_marker_obj is None:
            self.node.get_logger().info("No lane marker in object map! Failed.")
            return "failure"

        attempts = 0
        while (
            lane_marker_obj[4] is None
            or lane_marker_obj[5] is None
            and rclpy.ok()
        ):
            attempts += 1
            if attempts > 5:
                self.node.get_logger().info("Lane marker angle could not be measured! Failed.")
                return "failure"
            if self.timeout_occurred:
                return "timeout"
            self.node.get_clock().sleep_for(Duration(seconds=5))
            self.mapping.updateObject(lane_marker_obj)
            if self.timeout_occurred:
                return "timeout"
            self.control.move(
                (
                    lane_marker_obj[1],
                    lane_marker_obj[2],
                    self.node.get_parameter("down_cam_search_depth").get_parameter_value(),
                ),
                face_destination=True,
            )

        # Waiting 10 seconds and repeating to make sure it's correct
        self.node.get_logger().info("Waiting to ensure correct measurement of lane marker")
        self.mapping.updateObject(lane_marker_obj)
        if self.timeout_occurred:
            return "timeout"
        self.control.move(
            (
                lane_marker_obj[1],
                lane_marker_obj[2],
                self.node.get_parameter("down_cam_search_depth").get_parameter_value(),
            ),
            face_destination=True,
        )
        if self.timeout_occurred:
            return "timeout"
        self.node.get_clock().sleep_for(Duration(seconds=int(self.node.get_parameter("object_observation_time").get_parameter_value())))
        self.mapping.updateObject(lane_marker_obj)
        if self.timeout_occurred:
            return "timeout"
        self.control.move(
            (
                lane_marker_obj[1],
                lane_marker_obj[2],
                self.node.get_parameter("down_cam_search_depth").get_parameter_value(),
            ),
            face_destination=True,
        )

        heading1 = lane_marker_obj[4]
        heading2 = lane_marker_obj[5]

        # find heading which is pointing the least towards the AUV
        lane_marker_heading1_vec = normalize_vector(degreesToVector(heading1))
        lane_marker_heading2_vec = normalize_vector(degreesToVector(heading2))
        lane_marker_to_origin_vec = normalize_vector(
            (
                lane_marker_obj[1] - origin_position[0],
                lane_marker_obj[2] - origin_position[1],
            )
        )

        heading1_dot = dotProduct(lane_marker_to_origin_vec, lane_marker_heading1_vec)
        heading2_dot = dotProduct(lane_marker_to_origin_vec, lane_marker_heading2_vec)

        self.node.get_logger().info("Rotating to lane marker target heading.")

        #   rotate to that heading
        if self.timeout_occurred:
            return "timeout"
        if heading1_dot < heading2_dot:
            self.control.rotateEuler((0, 0, heading2))
        else:
            self.control.rotateEuler((0, 0, heading1))

        self.control.freeze_pose()
        self.thread_timer.cancel()
        self.node.get_logger().info("Successfully rotated to lane marker!")
        return "success"