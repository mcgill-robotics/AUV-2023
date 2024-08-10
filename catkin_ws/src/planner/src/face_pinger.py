#!/usr/bin/env python3
import rospy
from substates.utility.functions import countdown
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.functions import *


rospy.init_node("face_pinger")
print("_______SLEEPING__________")


pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)


def update_display(status):
    pub_mission_display.publish(status)
    print(status)


state = StateTracker()
control = Controller(rospy.Time(0))

while not rospy.is_shutdown():
    rospy.sleep(1)
    pinger_bearings = state.pingers_bearing
    if len(pinger_bearings.keys()) > 0:
        frequency, bearing = list(pinger_bearings.items())[0]
        update_display(str(frequency))
        control.rotateEuler(
            (
                0,
                0,
                180 + vectorToYawDegrees(bearing.x, bearing.y),
            )
        )
    else:
        update_display("NONE")
