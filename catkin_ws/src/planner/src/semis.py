#!/usr/bin/env python3
import rospy
from substates.utility.functions import countdown
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.functions import *


TARGET_FREQUENCY = 40000
PINGER_STEP_DISTANCE = 1

rospy.init_node("semis")
pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)


def update_display(status):
    pub_mission_display.publish(status)
    print(status)


countdown(30)

update_display("INIT")
state = StateTracker()
controls = Controller(rospy.Time(0))


update_display("MOVE2GATE")
controls.rotateDeltaEuler([0, 0, 0]) #TODO: change rotation based on flip coin
angle_to_gate = state.theta_z
controls.moveDelta([None, None, -0.75])
controls.moveDelta([1, None, None])


update_display("TRICKS")
for _ in range(2 * 3):
    controls.rotateDeltaEuler((0, 0, 120))


update_display("GATE")
controls.moveDelta([2, None, None])

update_display("PINGER")
previous_angle = None
while not rospy.is_shutdown():

    pinger_bearing = state.pingers_bearing.get(TARGET_FREQUENCY, None)

    if pinger_bearing is not None:
        current_angle = vectorToYawDegrees(pinger_bearing.x, pinger_bearing.y)
        if previous_angle is not None:
            angle_change = abs(current_angle - previous_angle)
            if angle_change > 180:
                angle_change -= 180
            if angle_change > 90:
                break

        print(f" >>>>>>>>>>>>>>>>> going to pinger @ {TARGET_FREQUENCY} Hz")
        controls.rotateEuler(
            (
                0,
                0,
                180 + current_angle,
            )
        )
        pinger_bearing_np = np.array([pinger_bearing.x, pinger_bearing.y])
        current_distance = np.linalg.norm(pinger_bearing_np)
        vector_auv_pinger = (
            -pinger_bearing_np / current_distance
        ) * PINGER_STEP_DISTANCE
        controls.moveDelta(vector_auv_pinger)
        previous_angle = current_angle

update_display("OCTAGON")
controls.rotateEuler((0, 0, angle_to_gate))
controls.moveDeltaLocal((0.25, 0, 0))
controls.rotateDeltaEuler((0, 0, -90))
controls.moveDeltaLocal((0.5, 0, 0))

controls.kill()
