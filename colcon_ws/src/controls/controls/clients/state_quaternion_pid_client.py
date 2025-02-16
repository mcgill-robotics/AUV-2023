#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from auv_msgs.action import EffortAction
from std_msgs.msg import Bool
from clients.base_client import SimpleClient


'''
Designed to be compatible with base_client.py's SimpleClient.
If functionality is to be expanded, it should probably be done there if it also
applies to effort stuff.

ROS2 version of actionlib.SimpleActionClient
'''

class StateClient(Node, SimpleClient):
    def __init__(self):
        super().__init__('state_client')
        
        # create an action client for the EffortAction
        self._action_client = ActionClient(self, 
                                           StateQuaternionAction, 
                                           '/controls/server/state')
        
    def send_goal(self):
        goal_msg = StateQuaternionAction.Goal()
        
        super.send_goal(goal_msg)   

        
    def send_goal_and_wait(self):
        goal_msg = StateQuaternionAction.Goal()
        
        super.send_goal_and_wait(goal_msg)
        

