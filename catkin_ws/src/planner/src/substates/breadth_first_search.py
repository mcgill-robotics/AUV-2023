#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time
import threading

#search for objects by moving in a growing square (i.e. each side of square grows in size after every rotation)
class BreadthFirstSearch(smach.State):
        ## NOTE: target classes should be an array of elements of the form (target_class, min_objs_required)
    def __init__(self, expansionAmt, control, mapping, target_class, min_objects, timeout):
        super().__init__(outcomes=['success', 'failure'])
        self.control = control
        self.mapping = mapping
        self.detectedObject = False
        self.timeout = timeout
        self.expansionAmt = expansionAmt
        self.target_class = target_class
        self.min_objects = min_objects

    def doBreadthFirstSearch(self):
        rotating = False
        moving = False
        def rotationComplete(self, msg): #called when rotation is complete
            global rotating
            rotating = False
        def movementComplete(self, msg): #called when translation is complete
            global moving
            moving = False
            
        movement = [1,0,0]
        right_turn = (0,0,-90)

        while True:
            #move forward
            print("Moving by {}.".format(movement))
            moving = True
            self.control.moveDeltaLocal(movement, movementComplete)
            #check for object detected while moving
            while moving:
                if self.detectedObject: return # stop grid search when object found
            #increase distance to move forward
            movement[0] += self.expansionAmt
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            rotating = True
            self.control.rotateDeltaEuler(right_turn, rotationComplete)
            #check for object detected while rotating
            while rotating:
                if self.detectedObject: return # stop grid search when object found

    def execute(self, ud):
        print("Starting breadth-first search.")
        try:
            self.searchThread = threading.Thread(target=self.doBreadthFirstSearch)
            self.searchThread.start()
            startTime = time.time()
            while startTime + self.timeout > time.time(): 
                if len(self.mapping.getClass(self.target_class)) >= self.min_objects:
                    self.detectedObject = True
                    self.searchThread.join()
                    self.control.stop_in_place()
                    print("Found object! Waiting 10 seconds to get more observations of object.")
                    rospy.sleep(10)
                    return 'success'
            print("Breadth-first search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.detectedObject = True
            self.control.stop_in_place()
            self.searchThread.join()
            print("Breadth-first search interrupted by user.")
            return 'failure'

