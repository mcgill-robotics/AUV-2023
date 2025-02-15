#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose, Vector3, Quaternion
import numpy as np
import quaternion


class QuaternionPID:

    def __init__(self):

        self.Kp = rospy.get_param("~Kp")
        self.Ki = rospy.get_param("~Ki")
        self.Kd = rospy.get_param("~Kd")

        self.windup_limit = rospy.get_param("~windup_limit")

        self.body_quat = np.quaternion(1, 0, 0, 0)
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.goal_quat = None
        self.enabled = False
        self.previous_time = rospy.get_time()
        self.torque_integral = np.array([0.0, 0.0, 0.0])

        self.pose_sub = rospy.Subscriber("/state/pose", Pose, self.set_pose)
        self.angular_velocity_sub = rospy.Subscriber(
            "/state/angular_velocity", Vector3, self.set_ang_vel
        )
        self.goal_sub = rospy.Subscriber(
            "/controls/pid/quat/setpoint", Quaternion, self.set_goal
        )
        self.enable_sub = rospy.Subscriber(
            "/controls/pid/quat/enable", Bool, self.set_enabled
        )

        self.pub_roll = rospy.Publisher("/controls/torque/roll", Float64, queue_size=1)
        self.pub_pitch = rospy.Publisher(
            "/controls/torque/pitch", Float64, queue_size=1
        )
        self.pub_yaw = rospy.Publisher("/controls/torque/yaw", Float64, queue_size=1)

        self.pub_error_quat = rospy.Publisher(
            "/controls/pid/quat/error", Float64, queue_size=1
        )

    def set_pose(self, data):
        self.body_quat = np.quaternion(
            data.orientation.w,
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
        )
        if self.body_quat.w < 0:
            self.body_quat = -self.body_quat

    def set_ang_vel(self, data):
        self.angular_velocity = np.array([data.x, data.y, data.z])

    def set_goal(self, data):
        self.goal_quat = np.quaternion(data.w, data.x, data.y, data.z)
        if self.goal_quat.w < 0:
            self.goal_quat = -self.goal_quat

        self.torque_integral = np.array([0, 0, 0])
        self.previous_time = rospy.get_time()

    def set_enabled(self, data):
        self.enabled = data.data

    def execute(self):
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if self.enabled and self.goal_quat is not None:
                self.Kp = rospy.get_param("~Kp")
                self.Ki = rospy.get_param("~Ki")
                self.Kd = rospy.get_param("~Kd")
                self.windup_limit = rospy.get_param("~windup_limit")
                roll_effort, pitch_effort, yaw_effort = self.controlEffort()
                self.pub_roll.publish(roll_effort)
                self.pub_pitch.publish(pitch_effort)
                self.pub_yaw.publish(yaw_effort)
            rate.sleep()

    def calculateQuatError(self, q1, q2):
        return q1.inverse() * q2

    def controlEffort(self):
        # Calculate error values
        error_quat = self.calculateQuatError(self.body_quat, self.goal_quat)
        self.pub_error_quat.publish(error_quat.w)

        if error_quat.w < 0:
            error_quat = -error_quat

        curr_time = rospy.get_time()
        delta_t = curr_time - self.previous_time
        self.previous_time = curr_time
        axis = np.array([error_quat.x, error_quat.y, error_quat.z])
        diff = axis * delta_t
        self.torque_integral = self.torque_integral + diff
        proportional_effort = np.zeros(3)
        if np.linalg.norm(self.torque_integral) > self.windup_limit:
            self.torque_integral = (
                self.windup_limit
                * self.torque_integral
                / np.linalg.norm(self.torque_integral)
            )

        proportional_effort[0] = self.Kp * error_quat.x
        proportional_effort[1] = self.Kp * error_quat.y
        proportional_effort[2] = self.Kp * error_quat.z

        # Calculate derivative term
        derivative_effort = self.Kd * self.angular_velocity

        integral_effort = self.Ki * self.torque_integral

        # Calculate integral term
        self.last_integral_time = rospy.get_time()

        control_effort = proportional_effort - derivative_effort + integral_effort

        # inertial_matrix = np.array([[0.042999259180866,  0.000000000000000, -0.016440893216213],
        #                             [0.000000000000000,  0.709487776484284, 0.003794052280665],
        #                             [-0.016440893216213, 0.003794052280665, 0.727193353794052]])

        inertial_matrix = np.array(
            [
                [1, 0.0, 0.0],
                [0.0, 1, 0.0],
                [0.0, 0.0, 0.5],
            ]
        )

        torque = np.matmul(inertial_matrix, control_effort)
        return torque


if __name__ == "__main__":
    rospy.init_node("quaternion_pid")
    pid = QuaternionPID()
    pid.execute()
    rospy.spin()
