#!/usr/bin/env python3

from auv_msgs.msg import PingerTimeDifference
import rospy

rospy.init_node("calibrate_dvl")
    
def cb_hydrophones_time_difference(msg):
    if msg.frequency != 0:
        print(msg)

sub = rospy.Subscriber("/sensors/hydrophones/pinger_time_difference", PingerTimeDifference, cb_hydrophones_time_difference)

rospy.spin()
