#!/usr/bin/env python3

import rclpy
import smach
from rclpy.clock import Clock
from rclpy import Duration
import threading
from std_msgs.msg import String


# search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class BreadthFirstSearch(smach.State):
    def __init__(self, node, control, mapping, target_class, min_objects):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.node = node
        self.clock = Clock()
        
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        self.expansionAmt = self.node.get_parameter("bfs_expansion_size").get_parameter_value()
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = self.node.get_parameter("object_search_time_limit").get_parameter_value()

        self.pub_mission_display = self.node.create_publisher(
            String, "/mission_display", 1
        )

    def do_breadth_first_search(self):
        movement = [0, self.expansionAmt, 0]
        while rclpy.ok():
            # move left
            self.node.get_logger().info("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject:
                return  # stop grid search when object found
            # move left by the same amount again
            self.node.get_logger().info("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject:
                return  # stop grid search when object found
            # increase distance to move by
            movement[1] += self.expansionAmt

    def timer_thread_func(self):
        self.pub_mission_display.publish("BFS Time-out")
        self.timeout_occurred = True

    def execute(self, _):
        self.node.get_logger().info("Starting breadth-first search.")
        self.pub_mission_display.publish("BFS Search")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, self.node.get_parameter("nominal_depth").get_parameter_value()))
        self.control.flatten()

        self.searchThread = threading.Thread(target=self.do_breadth_first_search)
        self.searchThread.start()

        while rclpy.ok():
            if self.timeout_occurred:
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                self.node.get_logger().info("Breadth-first search timed out.")
                return "timeout"
            elif len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.thread_timer.cancel()
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                self.node.get_logger().info("Found object! Waiting to get more observations of object.")
                self.node.get_clock().sleep_for(Duration(seconds=int(self.node.get_parameter("object_observation_time").get_parameter_value())))
                
                return "success"

        self.detectedObject = True
        self.searchThread.join()
        self.control.freeze_pose()
        self.node.get_logger().info("Breadth-first search failed.")
        return "failure"