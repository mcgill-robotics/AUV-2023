#!/usr/bin/env python3

import rospy
import smach
import threading
from std_msgs.msg import String


# ASSUMES AUV IS FACING DIRECTION TO SEARCH IN
class LinearSearch(smach.State):
    def __init__(self, control, mapping, target_class, min_objects):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("object_search_time_limit")
        
        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("LS Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, _):
        print("Starting linear search.")
        self.pub_mission_display.publish("Linear Search")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()
        
        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, rospy.get_param("nominal_depth")))
        self.control.flatten()

        while not rospy.is_shutdown():
            if self.timeout_occurred:
                print("Linear search timed out.")
                return "timeout"
            elif len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.thread_timer.cancel()
                self.detectedObject = True
                self.control.freeze_pose()
                print(
                    "Found object! Waiting 10 seconds to get more observations of object."
                )
                rospy.sleep(rospy.get_param("object_observation_time"))
                return "success"
            self.control.moveDeltaLocal(
                (rospy.get_param("linear_search_step_size"), 0, 0)
            )
            rospy.sleep(0.1)

        self.control.freeze_pose()
        print("Linear search failed.")
        return "failure"