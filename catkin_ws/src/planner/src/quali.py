#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
from substates.utility.functions import countdown

rospy.init_node("quali")
print("_______SLEEPING__________")
countdown(30)
controls = Controller(rospy.Time(0))
print("STARTING")
controls.rotateDeltaEuler([0,0,0])
# controls.freeze_pose()
# controls.freeze_position()
# controls.freeze_rotation()
print("ROTATE")
controls.moveDelta([None, None, -0.75])
print("MOVE DELTA")
controls.forceLocal([20,0])
print("FORCE")
rospy.sleep(10)

controls.kill()

