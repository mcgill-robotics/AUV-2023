#!/usr/bin/env python3

import rclpy
from rclpy import Duration
from rclpy.clock import Clock
import smach
import threading
from std_msgs.msg import String


# ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, node, control, mapping, target_class, min_objects):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.node = node
        
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = self.node.get_parameter("object_search_time_limit").get_parameter_value()
        
        self.pub_mission_display = self.node.create_publisher(
            String, "/mission_display", 1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("LS Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, _):
        self.get_logger().info("Starting linear search.")
        self.pub_mission_display.publish("Linear Search")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()
        
        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, self.node.get_parameter("nominal_depth").get_parameter_value()))
        self.control.flatten()

        while rclpy.ok():
            if self.timeout_occurred:
                self.get_logger().info("Linear search timed out.")
                return "timeout"
            elif len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.thread_timer.cancel()
                self.detectedObject = True
                self.control.freeze_pose()
                self.get_logger().info(
                    "Found object! Waiting 10 seconds to get more observations of object."
                )
                self.node.get_clock().sleep_for(Duration(seconds=int(self.node.get_parameter("object_observation_time").get_parameter_value())))
                return "success"
            self.control.moveDeltaLocal(
                (self.node.get_parameter("linear_search_step_size").get_parameter_value(), 0, 0)
            )
            self.node.get_clock().sleep_for(Duration(seconds= 0.1))

        self.control.freeze_pose()
        self.get_logger().info("Linear search failed.")
        return "failure"