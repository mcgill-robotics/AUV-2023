#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose, Quaternion, Vector3
import numpy as np

"""
This class servers as an abstract class for the action lib servers the controls use to
execute goals. In the process of developing the controls servers, I found that the I was
duplicating a lot of code so I made this base class to avoid issues of fixing a bug in one
server but not fixing it in another.

The methods this class have are mainly boiler plate. There are helper methods to establish
publishers and subscribers, a default preempt callback that sets the pids to the current position,
methods to check if a goal pose has been entered, a method to automatically turn off pids,
and a method to publish setpoints to the pids.
"""


class BaseServer:
    def __init__(self) -> None:
        self.cancelled = False
        self.goal = None
        self.pose = None
        self.body_quat = np.quaternion(1, 0, 0, 0)
        self.establish_effort_publishers()
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.establish_state_subscribers()

    def establish_effort_publishers(self):
        self.pub_surge = rospy.Publisher("/controls/force/surge", Float64, queue_size=1)
        self.pub_sway = rospy.Publisher("/controls/force/sway", Float64, queue_size=1)
        self.pub_heave = rospy.Publisher("/controls/force/heave", Float64, queue_size=1)
        self.pub_roll = rospy.Publisher("/controls/torque/roll", Float64, queue_size=1)
        self.pub_pitch = rospy.Publisher(
            "/controls/torque/pitch", Float64, queue_size=1
        )
        self.pub_yaw = rospy.Publisher("/controls/torque/yaw", Float64, queue_size=1)
        self.pub_global_x = rospy.Publisher(
            "/controls/force/global/x", Float64, queue_size=1
        )
        self.pub_global_y = rospy.Publisher(
            "/controls/force/global/y", Float64, queue_size=1
        )
        self.pub_global_z = rospy.Publisher(
            "/controls/force/global/z", Float64, queue_size=1
        )

    def establish_pid_publishers(self):
        self.pub_z_pid = rospy.Publisher(
            "/controls/pid/z/setpoint", Float64, queue_size=1
        )
        self.pub_y_pid = rospy.Publisher(
            "/controls/pid/y/setpoint", Float64, queue_size=1
        )
        self.pub_x_pid = rospy.Publisher(
            "/controls/pid/x/setpoint", Float64, queue_size=1
        )
        self.pub_quat_pid = rospy.Publisher(
            "/controls/pid/quat/setpoint", Quaternion, queue_size=1
        )

    def establish_pid_enable_publishers(self):
        self.pub_x_enable = rospy.Publisher(
            "/controls/pid/x/enable", Bool, queue_size=1
        )
        self.pub_y_enable = rospy.Publisher(
            "/controls/pid/y/enable", Bool, queue_size=1
        )
        self.pub_z_enable = rospy.Publisher(
            "/controls/pid/z/enable", Bool, queue_size=1
        )
        self.pub_quat_enable = rospy.Publisher(
            "/controls/pid/quat/enable", Bool, queue_size=1
        )

    def establish_state_subscribers(self):
        self.sub = rospy.Subscriber("/state/pose", Pose, self.set_pose)
        self.sub = rospy.Subscriber("/state/theta/x", Float64, self.set_theta_x)
        self.sub = rospy.Subscriber("/state/theta/y", Float64, self.set_theta_y)
        self.sub = rospy.Subscriber("/state/theta/z", Float64, self.set_theta_z)

    # callback for subscriber
    def set_pose(self, data):
        self.pose = data
        self.body_quat = np.quaternion(
            data.orientation.w,
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
        )
        if self.body_quat.w < 0:
            self.body_quat = -self.body_quat

    # callback for subscriber
    def set_theta_x(self, data):
        self.theta_x = data.data

    # callback for subscriber
    def set_theta_y(self, data):
        self.theta_y = data.data

    # callback for subscriber
    def set_theta_z(self, data):
        self.theta_z = data.data

    # generic cancel that publishes current position to pids to stay in place
    def cancel(self):
        self.cancelled = True

        self.pub_x_enable.publish(Bool(False))
        self.pub_y_enable.publish(Bool(False))
        self.pub_z_enable.publish(Bool(False))
        self.pub_quat_enable.publish(Bool(False))

        self.pub_global_x.publish(0)
        self.pub_global_y.publish(0)
        self.pub_global_z.publish(0)
        self.pub_roll.publish(0)
        self.pub_pitch.publish(0)
        self.pub_yaw.publish(0)
        self.pub_surge.publish(0)
        self.pub_sway.publish(0)
        self.pub_heave.publish(0)

        self.server.set_succeeded()
