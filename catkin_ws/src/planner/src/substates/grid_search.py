#!/usr/bin/env python3

import rospy
import smach
from .utility.vision import *
import time
import threading

#assumes AUV is facing direction it wants to search (i.e. grid will move in direction AUV is facing)
class GridSearch(smach.State):
    ## NOTE: target classes should be an array of elements of the form (target_class, min_objs_required)
    def __init__(self, timeout, target_classes=[], control=None, mapping=None):
        super().__init__(outcomes=['success', 'failure'])
        if control == None: raise ValueError("control argument is None")
        self.control = control
        if mapping == None: raise ValueError("mapping argument is None")
        self.mapping = mapping
        self.detectedObject = False
        self.target_classes = target_classes
        self.timeout = timeout

    def doGridSearch(self):
        rotating = False
        moving = False
        def rotationComplete(): #called when rotation is complete
            global rotating
            rotating = False
        def movementComplete(): #called when translation is complete
            global moving
            moving = False
            
        forward_movement = (1,0,0)
        side_movement = (2,0,0)
        right_turn = (0,0,-90)
        left_turn = (0,0,90)

        #SETUP (move AUV to the right to center grid search formation)

        #move forward
        print("Moving by {}.".format(forward_movement))
        moving = True
        self.control.moveDeltaLocal(forward_movement, movementComplete)
        #check for object detected while moving
        while moving:
            if self.detectedObject: return # stop grid search when object found
        #rotate right 90 degrees
        print("Rotating by {}.".format(right_turn))
        rotating = True
        self.control.rotateDelta(right_turn, rotationComplete)
        #check for object detected while rotating
        while rotating:
            if self.detectedObject: return # stop grid search when object found
        #move forward
        print("Moving by {}.".format(side_movement/2))
        moving = True
        self.control.moveDeltaLocal(side_movement, movementComplete)
        #check for object detected while moving
        while moving:
            if self.detectedObject: return # stop grid search when object found
        #turn left 90 degrees
        print("Rotating by {}.".format(left_turn))
        rotating = True
        self.control.rotateDelta(left_turn, rotationComplete)
        #check for object detected while rotating
        while rotating:
            if self.detectedObject: return # stop grid search when object found

        #REPEAT GRID SEARCH PATTERN
        while True:
            #move forward
            print("Moving by {}.".format(forward_movement))
            moving = True
            self.control.moveDeltaLocal(forward_movement, movementComplete)
            #check for object detected while moving
            while moving:
                if self.detectedObject: return # stop grid search when object found
            #turn left 90 degrees
            print("Rotating by {}.".format(left_turn))
            rotating = True
            self.control.rotateDelta(left_turn, rotationComplete)
            #check for object detected while rotating
            while rotating:
                if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {}.".format(side_movement))
            moving = True
            self.control.moveDeltaLocal(side_movement, movementComplete)
            #check for object detected while moving
            while moving:
                if self.detectedObject: return # stop grid search when object found
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            rotating = True
            self.control.rotateDelta(right_turn, rotationComplete)
            #check for object detected while rotating
            while rotating:
                if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {}.".format(forward_movement))
            moving = True
            self.control.moveDeltaLocal(forward_movement, movementComplete)
            #check for object detected while moving
            while moving:
                if self.detectedObject: return # stop grid search when object found
            #rotate right 90 degrees
            print("Rotating by {}.".format(right_turn))
            rotating = True
            self.control.rotateDelta(right_turn, rotationComplete)
            #check for object detected while rotating
            while rotating:
                if self.detectedObject: return # stop grid search when object found
            #move forward
            print("Moving by {}.".format(side_movement))
            moving = True
            self.control.moveDeltaLocal(side_movement, movementComplete)
            #check for object detected while moving
            while moving:
                if self.detectedObject: return # stop grid search when object found
            #turn left 90 degrees
            print("Rotating by {}.".format(left_turn))
            rotating = True
            self.control.rotateDelta(left_turn, rotationComplete)
            #check for object detected while rotating
            while rotating:
                if self.detectedObject: return # stop grid search when object found

    def execute(self, ud):
        print("Starting grid search.")
        try:
            self.searchThread = threading.Thread(target=self.doGridSearch)
            self.searchThread.start()
            startTime = time.time()
            while startTime + self.timeout > time.time(): 
                for cls, min_objs in self.target_classes:
                    if len(self.mapping.getClass(cls)) >= min_objs:
                        self.detectedObject = True
                        self.control.stop_in_place()
                        self.searchThread.join()
                        print("Found object!")
                        return 'success'
            print("Grid search timed out.")
            return 'failure'
        except KeyboardInterrupt:
            self.detectedObject = True
            self.control.stop_in_place()
            self.searchThread.join()
            print("Grid search interrupted by user.")
            return 'failure'

