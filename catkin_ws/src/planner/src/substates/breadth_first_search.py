#!/usr/bin/env python3

import rospy
import smach
import threading
from std_msgs.msg import String


# search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class BreadthFirstSearch(smach.State):
    def __init__(self, control, mapping, target_class, min_objects):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        
        self.detectedObject = False
        self.target_class = target_class
        self.min_objects = min_objects
        self.expansionAmt = rospy.get_param("bfs_expansion_size")
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("object_search_time_limit")

        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def do_breadth_first_search(self):
        movement = [0, self.expansionAmt, 0]
        while not rospy.is_shutdown():
            # move left
            print("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject:
                return  # stop grid search when object found
            # move left by the same amount again
            print("Moving by {}.".format(movement))
            self.control.moveDeltaLocal(movement, face_destination=True)
            if self.detectedObject:
                return  # stop grid search when object found
            # increase distance to move by
            movement[1] += self.expansionAmt

    def timer_thread_func(self):
        self.pub_mission_display.publish("BFS Time-out")
        self.timeout_occurred = True

    def execute(self, _):
        print("Starting breadth-first search.")
        self.pub_mission_display.publish("BFS Search")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, rospy.get_param("nominal_depth")))
        self.control.flatten()

        self.searchThread = threading.Thread(target=self.do_breadth_first_search)
        self.searchThread.start()

        while not rospy.is_shutdown():
            if self.timeout_occurred:
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                print("Breadth-first search timed out.")
                return "timeout"
            elif len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                self.thread_timer.cancel()
                self.detectedObject = True
                self.searchThread.join()
                self.control.freeze_pose()
                print("Found object! Waiting to get more observations of object.")
                rospy.sleep(rospy.get_param("object_observation_time"))
                return "success"

        self.detectedObject = True
        self.searchThread.join()
        self.control.freeze_pose()
        print("Breadth-first search failed.")
        return "failure"