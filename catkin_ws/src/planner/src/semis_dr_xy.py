#!/usr/bin/env python3
import rospy
from substates.utility.functions import countdown
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.functions import *


START2OCT_X = 1.4 + 2.74 * 6.5
START2OCT_Y = 1.5 * 2.74

rospy.init_node("semis")
pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)


def update_display(status):
    pub_mission_display.publish(status)
    print(status)


countdown(120)

update_display("INIT")
state = StateTracker()
controls = Controller(rospy.Time(0))


update_display("MOVE2GATE")
controls.rotateDeltaEuler([0, 0, 0])  # TODO: change rotation based on flip coin
angle_to_gate = state.theta_z
controls.moveDelta([None, None, -0.75])
controls.moveDeltaLocal([1, None, None])


update_display("TRICKS")
for _ in range(2 * 3):
    controls.rotateDeltaEuler((0, 0, 120))


update_display("GATE + NAV TO OCT")
controls.moveDeltaLocal([START2OCT_X - 1, 0, 0])
controls.rotateDeltaEuler((0, 0, -90))
controls.moveDeltaLocal([START2OCT_Y, 0, 0])

controls.kill()
