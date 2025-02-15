#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock
from rclpy import Duration
import smach
from .utility.functions import *
from auv_msgs.msg import PingerBearing
import threading
from std_msgs.msg import String


# Given a pinger number (integer), navigate the AUV towards the object corresponding to that pinger number
class NavigatePinger(smach.State):

    def __init__(
        self, node, control, state, mapping, pinger_frequency, update_heading_time, advance_distance, goal_object
    ):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.node = node
        # An integer from 0 to 3 corresponding to a specifc pinger & object
        self.pinger_frequency = pinger_frequency # change to freq
        self.update_heading_time = update_heading_time
        self.goal_object = goal_object
        self.advance_distance = advance_distance
        self.nominal_depth = self.node.get_parameter("nominal_depth").get_parameter_value()

        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = self.node.get_parameter("navigate_pinger_time_limit").get_parameter_value()

        self.pub_mission_display = self.node.create_publisher(
            "/mission_display", String, queue_size=1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("Pinger Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, ud):
        self.node.get_logger().info("Starting navigate pinger.")
        self.pub_mission_display.publish("Pinger")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, self.node.get_parameter("nominal_depth").get_parameter_value()))
        self.control.flatten()

        current_pinger = self.state.pingers_bearing[self.pinger_frequency]
        current_pinger_bearing = np.array([current_pinger.x, current_pinger.y, current_pinger.z])
        self.node.get_logger().info("Current pinger bearing: ", current_pinger_bearing)

        if current_pinger_bearing is None:
            self.node.get_logger().info("No pinger data!")
            return "failure"

        self.node.get_logger().info("Centering and rotating in front of pinger.")

        if self.timeout_occurred:
            return "timeout"

        pinger_bearings_sum = np.zeros(3)
        number_pinger_measurements = 0
        reset_time = Clock().now().seconds_nanoseconds()[0]

        while True:
            current_pinger = self.state.pingers_bearing[self.pinger_frequency]
            current_pinger_bearing = np.array([current_pinger.x, current_pinger.y, current_pinger.z])
            pinger_bearings_sum += current_pinger_bearing
            number_pinger_measurements += 1

            if (Clock().now().seconds_nanoseconds()[0] - reset_time) >= self.update_heading_time:
                pinger_bearings_average = (
                    pinger_bearings_sum / number_pinger_measurements
                )
               
                # Move towards the Pinger 
                self.node.get_logger().info("Moving towards pinger")
                self.node.get_logger().info("Pinger bearings average: ", pinger_bearings_average)
                self.control.rotateEuler((0,0, 180 + vectorToYawDegrees(pinger_bearings_average[0], pinger_bearings_average[1])))
                
                current_distance = np.linalg.norm(pinger_bearings_average)
                vector_auv_pinger = -pinger_bearings_average / current_distance * self.advance_distance
                
                self.control.moveDelta(vector_auv_pinger)
                
                reset_time = Clock().now().seconds_nanoseconds()[0]
                
                if self.timeout_occurred:
                    return "timeout"
                
                # After we move we look for object, if it has been found then success
                object = self.mapping.getClosestObject(
                    cls=self.goal_object, pos=(self.state.x, self.state.y)
                )
                if object is not None:
                    self.node.get_logger().info("The " + self.goal_object + " has been found. The mission was a success")
                    return "success"
                
