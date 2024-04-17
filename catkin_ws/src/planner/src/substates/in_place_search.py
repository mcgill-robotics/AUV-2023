#!/usr/bin/env python3

import rospy
import smach
import time
import threading
from std_msgs.msg import String

#search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class InPlaceSearch(smach.State):
    def __init__(self, control, mapping, target_class, min_objects):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.timeout = rospy.get_param("object_search_timeout")
        self.target_class = target_class
        self.min_objects = min_objects
        self.detectedObject = False
        self.pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)
        

    def doRotation(self):
        turn_amt = (0,0,rospy.get_param("in_place_search_rotation_increment"))
        num_turns = 0
        num_full_turns = 0

        while not rospy.is_shutdown():
            if self.preempt_requested():
                break
            if (num_turns >= 360/abs(turn_amt[2])):
                num_turns = 0
                num_full_turns += 1
                if num_full_turns == 1: self.control.move((None,None, rospy.get_param("nominal_depth") + 1))
                elif num_full_turns == 2: self.control.move((None,None, rospy.get_param("nominal_depth") - 1))
                else: return
            #move forward
            print("Rotating by {}.".format(turn_amt))
            self.control.rotateDeltaEuler(turn_amt)
            if self.detectedObject: return # stop grid search when object found
            num_turns += 1

    def execute(self, ud):
        print("Starting in-place search.")
        if self.target_class == "Gate":
            self.pub_mission_display.publish("Gate")
        else:
            self.pub_mission_display.publish("Buoy")

        #MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, rospy.get_param("nominal_depth")))
        self.control.flatten()

        self.searchThread = threading.Thread(target=self.doRotation)
        self.searchThread.start()
        print("Starting rotation.")
        startTime = time.time()
        while startTime + self.timeout > time.time() and not rospy.is_shutdown(): 
            if self.preempt_requested():
                print("IPS being preempted")
                self.service_preempt()
                return 'failure'
            if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                print("Found object! Waiting to get more observations of object.")
                rospy.sleep(rospy.get_param("object_observation_time"))
                return 'success'
        self.detectedObject = True
        self.searchThread.join()
        self.control.freeze_pose()
        print("In-place search timed out.")
        return 'failure'
