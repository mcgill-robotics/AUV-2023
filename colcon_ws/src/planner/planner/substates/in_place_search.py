#!/usr/bin/env python3

import rclpy
from rclpy import Duration
import smach
import threading
from std_msgs.msg import String


# search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class InPlaceSearch(smach.State):
    def __init__(self, node, control, mapping, target_class, min_objects):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.node = node

        self.target_class = target_class
        self.min_objects = min_objects
        self.detectedObject = False
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.TIME_LIMIT = self.node.get_parameter("object_search_time_limit").get_parameter_value()
        self.NOMINAL_DEPTH = self.node.get_parameter("nominal_depth").get_parameter_value()
        self.ROTATION_INCREMENT = self.node.get_parameter("in_place_search_rotation_increment").get_parameter_value()
        
        self.pub_mission_display = self.node.create_publisher(
            String, "/mission_display", 1
        )

    def doRotation(self):
        turn_amt = (0, 0, self.ROTATION_INCREMENT)
        num_turns = 0
        num_full_turns = 0

        while rclpy.ok():
            if num_turns >= 360 / abs(turn_amt[2]):
                num_turns = 0
                num_full_turns += 1
                if num_full_turns == 1:
                    self.control.move(
                        (None, None, self.NOMINAL_DEPTH + 1)
                    )
                elif num_full_turns == 2:
                    self.control.move(
                        (None, None, self.NOMINAL_DEPTH - 1)
                    )
                else:
                    return
            if self.detectedObject:
                return  # Stop grid search when object found.
            self.node.get_logger().info("Rotating by {}.".format(turn_amt))
            self.control.rotateDeltaEuler(turn_amt)
            num_turns += 1
            self.node.get_clock().sleep_for(Duration(seconds=5))

    def timer_thread_func(self):
        self.pub_mission_display.publish("IPS Time-out")
        self.node.get_logger().info("In-place search timed out. Moving back to nominal depth and flattening.")
        self.timeout_occurred = True
        self.control.preemptCurrentAction()
        self.control.move((None, None, self.NOMINAL_DEPTH))
        self.control.flattening()        

    def execute(self, _):
        self.node.get_logger().info("Starting in-place search.")
        self.pub_mission_display.publish("In-Place Search")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.TIME_LIMIT, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, self.NOMINAL_DEPTH))
        self.control.flatten()

        self.searchThread = threading.Thread(target=self.doRotation)
        self.searchThread.start()

        while rclpy.ok():
            if self.timeout_occurred:
                self.detectedObject = False
                self.searchThread.join()
                return "timeout"
            elif len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.thread_timer.cancel()
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                self.node.get_logger().info("Found object! Waiting to get more observations of object.")
                self.node.get_clock().sleep_for(Duration(seconds=int(self.node.get_parameter("object_observation_time").get_parameter_value())))
        
                return "success"
            
        self.detectedObject = False
        self.searchThread.join()
        self.control.freeze_pose()
        self.node.get_logger().info("In-place search failed.")
        return "failure"