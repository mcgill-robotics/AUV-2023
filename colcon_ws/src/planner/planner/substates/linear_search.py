#!/usr/bin/env python3

import rclpy
from rclpy import Duration
from rclpy.clock import Clock
import smach
import threading
from std_msgs.msg import String


# ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, control, mapping, target_class, min_objects, node):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.node = node
        
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rclpy.get_param("object_search_time_limit")
        
        self.pub_mission_display = rclpy.create_publisher(
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
        self.control.move((None, None, rclpy.get_param("nominal_depth")))
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
                self.node.get_clock().sleep_for(Duration(seconds=int(rclpy.get_param("object_observation_time"))))
                return "success"
            self.control.moveDeltaLocal(
                (rclpy.get_param("linear_search_step_size"), 0, 0)
            )
            self.node.get_clock().sleep_for(Duration(seconds= 0.1))

        self.control.freeze_pose()
        self.get_logger().info("Linear search failed.")
        return "failure"