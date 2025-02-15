#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose
from auv_msgs.msg import PingerBearing
import numpy as np
import quaternion

class StateTracker:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.pose = None
        self.quat = None    
        self.pingers_bearing = {}
        self.x_pos_sub = rospy.Subscriber('/state/x', Float64, self.updateX)
        self.y_pos_sub = rospy.Subscriber('/state/y', Float64, self.updateY)
        self.z_pos_sub = rospy.Subscriber('/state/z', Float64, self.updateZ)
        self.theta_x_sub = rospy.Subscriber('/state/theta/x', Float64, self.updateThetaX)
        self.theta_y_sub = rospy.Subscriber('/state/theta/y', Float64, self.updateThetaY)
        self.theta_z_sub = rospy.Subscriber('/state/theta/z', Float64, self.updateThetaZ)
        self.pose_sub = rospy.Subscriber('/state/pose', Pose, self.updatePose)
        self.hydrophone_sub = rospy.Subscriber('/sensors/hydrophones/pinger_bearing', PingerBearing, self.updatePingerBearing)
        self.claw_contact_sub = rospy.Subscriber("/actuators/grabber/contact", Bool, self.updateGrabberContact)

    def updatePose(self,msg):
        self.pose = msg
        self.quat = np.quaternion(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)

    def updateX(self, msg):
        self.x = float(msg.data)

    def updateY(self, msg):
        self.y = float(msg.data)

    def updateZ(self, msg):
        self.z = float(msg.data)

    def updateThetaX(self, msg):
        self.theta_x = float(msg.data)

    def updateThetaY(self, msg):
        self.theta_y = float(msg.data)

    def updateThetaZ(self, msg):
        self.theta_z = float(msg.data)
    
    def updatePingerBearing(self, msg):
        self.pingers_bearing[msg.frequency] = msg.pinger_bearing

    def updateGrabberContact(self, msg):
        self.grabber_contact = msg.data

    def stop(self):
        self.x_pos_sub.unregister()
        self.y_pos_sub.unregister()
        self.z_pos_sub.unregister()
        self.theta_x_sub.unregister()
        self.theta_y_sub.unregister()
        self.theta_z_sub.unregister()
        self.hydrophone_sub.unregister()
        self.claw_contact_sub.unregister()
        