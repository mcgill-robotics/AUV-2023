#!/usr/bin/env python3

import rclpy
import smach
from .utility.functions import *


class Quali(smach.State):
     def __init__(self, node, control):
          super().__init__(outcomes=["success", "failure", "timeout"])
          self.control = control
          self.node = node
          self.quali_gate_width = self.node.get_parameter("quali_gate_width").get_parameter_value()
          

     def execute(self, ud):
          self.node.get_logger().info("Starting quali.")

          self.node.get_logger().info("Moving to right side of gate")
          self.control.moveDeltaLocal((0, -self.quali_gate_width / 4, 0))

          self.node.get_logger().info("Moving through gate")
          self.control.moveDeltaLocal((14, 0, 0))

          self.node.get_logger().info("Rotating around pole")
          self.control.rotateDeltaEuler((0, 0, 90))

          self.control.moveDeltaLocal((self.quali_gate_width / 2, 0, 0))

          self.control.rotateDeltaEuler((0, 0, 90))

          self.node.get_logger().info("Returning to origin")
          self.control.moveDeltaLocal((17, 0, 0))

          return "success"
