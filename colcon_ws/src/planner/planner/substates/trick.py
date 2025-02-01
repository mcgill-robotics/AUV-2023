#!/usr/bin/env python3

import rclpy
import smach
import threading
from std_msgs.msg import String

class Trick(smach.State):
    def __init__(self, control, node):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.node = node
        self.control = control
        self.num_full_spins = self.node.get_parameter("num_full_spins").get_parameter_value()

        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = self.node.get_parameter("trick_time_limit").get_parameter_value()

        self.pub_mission_display = self.node.Publisher("/mission_display", String, queue_size=1)

    def timer_thread_func(self):
        self.pub_mission_display.publish("Gate Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self,ud):
        self.node.get_logger().info("Starting tricks...")
        self.pub_mission_display.publish("Trick") 

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        #STAY IN SAME POSITION AND AT FLAT ORIENTATION 
        self.control.freeze_position()
        self.control.flatten()

        for _ in range(self.num_full_spins*3): 
            if self.timeout_occurred:
                return "timeout"
            self.control.rotateDeltaEuler((120.0, 0, 0))
        self.node.get_logger().info("Completed")

        #re-stabilize
        self.node.get_logger().info("Stabilizing...")
        self.control.flatten()
        
        return "success"