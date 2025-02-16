#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.action.server import GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import time
import threading
from servers.base_server import BaseServer
from auv_msgs.msg import EffortAction
from std_msgs.msg import Bool

"""
Abstract class for the Effort server. The Effort server
accepts a EffortGoal which has an effort to exert on the auv
via the thrusters. The preempt callback is the default cancel method
which sets the pids to the current position.
"""


class EffortServer(BaseServer):
    def __init__(self, node_name_used):
        super().__init__(node_name_used)

        self.goal_handle_ : ServerGoalHandle = None #initializing a goal handle that can be used later


        self.server = ActionServer(
            self,
            EffortAction,
            "/controls/server/effort",
            EffortAction,
            execute_cb=self.callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )
    
    def goal_callback(self, goal_request: EffortAction.Goal):  
        # abort previous goal when getting new goal
        if self.goal_handle_ is not None and self.goal_handle_.is_active:
            self.goal_handle_.abort()
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """
        Executes a Effort Goal. Sets the efforts to the goal efforts.
        """
        self.goal_handle_ = goal_handle  # goal handle is a variable sent by the client 
        self.get_logger().info("Effort goal:\n", self.goal_handle_)
        self.unset_pids()

        if goal_handle.request.do_surge.data:
            self.pub_surge.publish(goal_handle.request.effort.force.x)
        if goal_handle.request.do_sway.data:
            self.pub_sway.publish(goal_handle.request.effort.force.y)
        if goal_handle.request.do_heave.data:
            self.pub_heave.publish(goal_handle.request.effort.force.z)

        if goal_handle.request.do_roll.data:
            self.pub_roll.publish(goal_handle.request.effort.torque.x)
        if goal_handle.request.do_pitch.data:
            self.pub_pitch.publish(goal_handle.request.effort.torque.y)
        if goal_handle.request.do_yaw.data:
            self.pub_yaw.publish(goal_handle.request.effort.torque.z)

        # Set goal final state as success 
        goal_handle.succeed()

    def unset_pids(self, goal_handle: ServerGoalHandle):
        """
        Unsets the pids for the given goal.
        """
        # if(goal_handle.request.do_roll.data): self.pub_theta_x_enable.publish(Bool(False))
        # if(goal_handle.request.do_pitch.data): self.pub_theta_y_enable.publish(Bool(False))
        # if(goal_handle.request.do_yaw.data): self.pub_theta_z_enable.publish(Bool(False))

        if goal_handle.request.do_surge.data or goal_handle.request.do_sway.data:
            self.pub_x_enable.publish(Bool(False))
            self.pub_global_x.publish(0)
            self.pub_y_enable.publish(Bool(False))
            self.pub_global_y.publish(0)

        if goal_handle.request.do_heave.data:
            self.pub_z_enable.publish(Bool(False))
            self.pub_global_z.publish(0)

        if goal_handle.request.do_roll.data or goal_handle.request.do_pitch.data or goal_handle.request.do_yaw.data:
            self.pub_quat_enable.publish(Bool(False))
