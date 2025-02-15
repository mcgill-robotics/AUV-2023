#!/usr/bin/env python3
import rospy
import smach
from .utility.functions import *
import threading
from std_msgs.msg import String


class NavigateDropper(smach.State):
    def __init__(self, control, mapping, state):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("navigate_bin_time_limit")
        self.centering_dist_threshold = rospy.get_param("center_dist_threshold")
        self.centering_delta_increment = rospy.get_param("centering_delta_increment")
        
        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("Bin Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, ud):
        print("Starting Bin Navigation.") 
        self.pub_mission_display.publish("Bin")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientation.
        self.control.flatten()

        bin_object = self.mapping.getClosestObject(cls="Bin", pos=(self.state.x, self.state.y))
        if bin_object is None:
            print("No Bin in object map! Failed.")
            return "failure"

        # Move to center of bin
        print("Moving to the center of the bin.")
        if self.timeout_occurred:
            return "timeout"
        self.control.move((bin_object[1], bin_object[2], rospy.get_param("down_cam_search_depth")), face_destination=True)
        # center distance loop
        while (self.mapping.distance > self.centering_dist_threshold):
            if self.timeout_occurred:
                return "timeout"
            if self.mapping.delta_height < 0:
                self.control.moveDeltaLocal((self.centering_delta_increment, 0, 0))
            elif self.mapping.delta_height > 0:
                self.control.moveDeltaLocal((-self.centering_delta_increment, 0, 0))
            if self.mapping.delta_width < 0:
                self.control.moveDeltaLocal((0, self.centering_delta_increment, 0))
            elif self.mapping.delta_width > 0:
                self.control.moveDeltaLocal((0, -self.centering_delta_increment, 0))
            rospy.sleep(3)
        print("Centered")
        
        # Flatten on top of bin
        if self.timeout_occurred:
            return "timeout"
        self.control.flatten()
        
        #dropping the ball once colour red is detected
        if self.timeout_occurred:
            return "timeout"
        print("Dropping ball.")
        self.control.open_claw()
        rospy.sleep(1)
        self.control.close_claw()
        print("Successfully dropped ball.")
        return 'success'