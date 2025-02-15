#!/usr/bin/env python3

import rospy
import smach
from .utility.functions import *


class Quali(smach.State):
     def __init__(self, control):
          super().__init__(outcomes=["success", "failure", "timeout"])
          self.control = control
          self.quali_gate_width = rospy.get_param("quali_gate_width")

     def execute(self, ud):
          print("Starting quali.")

          print("Moving to right side of gate")
          self.control.moveDeltaLocal((0, -self.quali_gate_width / 4, 0))

          print("Moving through gate")
          self.control.moveDeltaLocal((14, 0, 0))

          print("Rotating around pole")
          self.control.rotateDeltaEuler((0, 0, 90))

          self.control.moveDeltaLocal((self.quali_gate_width / 2, 0, 0))

          self.control.rotateDeltaEuler((0, 0, 90))

          print("Returning to origin")
          self.control.moveDeltaLocal((17, 0, 0))

          return "success"
