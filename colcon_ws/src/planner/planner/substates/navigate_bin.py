#!/usr/bin/env python3
import rclpy
from rclpy import Duration
from rclpy.clock import Clock
import smach
from .utility.functions import *
import threading
from std_msgs.msg import String


class NavigateDropper(smach.State):
    def __init__(self, control, mapping, state, node):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.node = node
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = self.node.get_parameter("navigate_bin_time_limit").get_parameter_value()
        self.centering_dist_threshold = self.node.get_parameter("center_dist_threshold").get_parameter_value()
        self.centering_delta_increment = self.node.get_parameter("centering_delta_increment").get_parameter_value()
        
        self.pub_mission_display = self.node.create_publisher(
             String, "/mission_display", 1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("Bin Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, ud):
        self.node.get_logger().info("Starting Bin Navigation.") 
        self.pub_mission_display.publish("Bin")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientation.
        self.control.flatten()

        bin_object = self.mapping.getClosestObject(cls="Bin", pos=(self.state.x, self.state.y))
        if bin_object is None:
            self.node.get_logger().info("No Bin in object map! Failed.")
            return "failure"

        # Move to center of bin
        self.node.get_logger().info("Moving to the center of the bin.")
        if self.timeout_occurred:
            return "timeout"
        self.control.move((bin_object[1], bin_object[2], self.node.get_parameter("down_cam_search_depth").get_parameter_value()), face_destination=True)
        # center distance loop
        while (self.mapping.distance > self.centering_dist_threshold):
            if self.timeout_occurred:
                return "timeout"
            if self.mapping.delta_height < 0:
                self.control.moveDeltaLocal((self.centering_delta_increment, 0, 0))
            elif self.mapping.delta_height > 0:
                self.control.moveDeltaLocal((-self.centering_delta_increment, 0, 0))
            if self.mapping.delta_width < 0:
                self.control.moveDeltaLocal((0, self.centering_delta_increment, 0))
            elif self.mapping.delta_width > 0:
                self.control.moveDeltaLocal((0, -self.centering_delta_increment, 0))
            self.node.get_clock().sleep_for(Duration(seconds=3))
        self.node.get_logger().info("Centered")
        
        # Flatten on top of bin
        if self.timeout_occurred:
            return "timeout"
        self.control.flatten()
        
        #dropping the ball once colour red is detected
        if self.timeout_occurred:
            return "timeout"
        self.node.get_logger().info("Dropping ball.")
        self.control.open_claw()
        self.node.get_clock().sleep_for(Duration(seconds=1))
        self.control.close_claw()
        self.node.get_logger().info("Successfully dropped ball.")
        return 'success'