#!/usr/bin/env python3

import rospy

import actionlib
from auv_msgs.msg import EffortAction
from std_msgs.msg import Bool
from servers.base_server import BaseServer

"""
Abstract class for the Effort server. The Effort server
accepts a EffortGoal which has an effort to exert on the auv
via the thrusters. The preempt callback is the default cancel method
which sets the pids to the current position.
"""


class EffortServer(BaseServer):
    def __init__(self):
        super().__init__()
        self.server = actionlib.SimpleActionServer(
            "/controls/server/effort",
            EffortAction,
            execute_cb=self.callback,
            auto_start=False,
        )
        self.server.start()

    def callback(self, goal):
        """
        Executes a Effort Goal. Sets the efforts to the goal efforts.
        """
        print("Effort goal:\n", goal)
        self.goal = goal
        self.unset_pids()

        if self.goal.do_surge.data:
            self.pub_surge.publish(self.goal.effort.force.x)
        if self.goal.do_sway.data:
            self.pub_sway.publish(self.goal.effort.force.y)
        if self.goal.do_heave.data:
            self.pub_heave.publish(self.goal.effort.force.z)

        if self.goal.do_roll.data:
            self.pub_roll.publish(self.goal.effort.torque.x)
        if self.goal.do_pitch.data:
            self.pub_pitch.publish(self.goal.effort.torque.y)
        if self.goal.do_yaw.data:
            self.pub_yaw.publish(self.goal.effort.torque.z)

        self.server.set_succeeded()

    def unset_pids(self):
        """
        Unsets the pids for the given goal.
        """
        # if(self.goal.do_roll.data): self.pub_theta_x_enable.publish(Bool(False))
        # if(self.goal.do_pitch.data): self.pub_theta_y_enable.publish(Bool(False))
        # if(self.goal.do_yaw.data): self.pub_theta_z_enable.publish(Bool(False))

        if self.goal.do_surge.data or self.goal.do_sway.data:
            self.pub_x_enable.publish(Bool(False))
            self.pub_global_x.publish(0)
            self.pub_y_enable.publish(Bool(False))
            self.pub_global_y.publish(0)

        if self.goal.do_heave.data:
            self.pub_z_enable.publish(Bool(False))
            self.pub_global_z.publish(0)

        if self.goal.do_roll.data or self.goal.do_pitch.data or self.goal.do_yaw.data:
            self.pub_quat_enable.publish(Bool(False))
