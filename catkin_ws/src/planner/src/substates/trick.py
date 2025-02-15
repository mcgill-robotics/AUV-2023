#!/usr/bin/env python3

import rospy
import smach
import threading
from std_msgs.msg import String

class Trick(smach.State):
    def __init__(self, control):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.num_full_spins = rospy.get_param("num_full_spins")

        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("trick_time_limit")

        self.pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)

    def timer_thread_func(self):
        self.pub_mission_display.publish("Gate Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self,ud):
        print("Starting tricks...")
        self.pub_mission_display.publish("Trick") 

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        #STAY IN SAME POSITION AND AT FLAT ORIENTATION 
        self.control.freeze_position()
        self.control.flatten()

        for _ in range(self.num_full_spins*3): 
            if self.timeout_occurred:
                return "timeout"
            self.control.rotateDeltaEuler((120.0, 0, 0))
        print("Completed")

        #re-stabilize
        print("Stabilizing...")
        self.control.flatten()
        
        return "success"