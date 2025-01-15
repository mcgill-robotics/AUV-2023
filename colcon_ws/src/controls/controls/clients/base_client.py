import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus


"""
This class clients is meant to control the functionalities of the client: getting goals, sending goals,
and cancelling goals

This is an alternative to the actionlib.SimpleClient which does not exist in ROS2 (as far as i know)
"""

class SimpleClient(Node):
    def __init__(self, node_name, name_of_server, data_types):
        super.__init__(node_name)
        self.client = ActionClient(self,
            data_types,
            name_of_server                       
        )
    
    def send_goal(self, goal_to_send):
        # Wait for server
        self.client.wait_for_server()

        #Send the goal
        self.client.send_goal_async(goal_to_send)

    def cancel_goal(self):
        self.goal_handle_.cancel_goal_async()