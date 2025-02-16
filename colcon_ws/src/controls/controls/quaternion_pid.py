#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Pose, Vector3, Quaternion
import numpy as np
import quaternion


class QuaternionPID(Node):

    def __init__(self):
        super().__init__('quaternion_pid')

        # get params
        self.Kp = self.declare_parameter("Kp").value
        self.Ki = self.declare_parameter("Ki").value
        self.Kd = self.declare_parameter("Kd").value
        self.windup_limit = self.declare_parameter("windup_limit").value

        # init vars
        self.body_quat = np.quaternion(1, 0, 0, 0)
        self.angular_velocity = np.array([0.0, 0.0, 0.0])
        self.goal_quat = None
        self.enabled = False
        self.previous_time = self.get_clock().now().nanoseconds / 1e9
        self.torque_integral = np.array([0.0, 0.0, 0.0])

        # create subscriptions
        self.pose_sub = self.create_subscription(Pose, "/state/pose", self.set_pose, 10)
        self.angular_velocity_sub = self.create_subscription(
            Vector3, "/state/angular_velocity", self.set_ang_vel, 10
        )
        self.goal_sub = self.create_subscription(
            Quaternion, "/controls/pid/quat/setpoint", self.set_goal, 10
        )
        self.enable_sub = self.create_subscription(
            Bool, "/controls/pid/quat/enable", self.set_enabled, 10
        )

        # create publishers
        self.pub_roll = self.create_publisher(Float64, "/controls/torque/roll", 10)
        self.pub_pitch = self.create_publisher(Float64, "/controls/torque/pitch", 10)
        self.pub_yaw = self.create_publisher(Float64, "/controls/torque/yaw", 10)
        self.pub_error_quat = self.create_publisher(Float64, "/controls/pid/quat/error", 10)

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
        self.previous_time = self.get_clock().now().nanoseconds / 1e9

    def set_enabled(self, data):
        self.enabled = data.data

    def execute(self):
        # execute the PID control in a loop at 100Hz
        rate = self.create_rate(100)

        while rclpy.ok():
            if self.enabled and self.goal_quat is not None:
                self.Kp = self.declare_parameter("Kp").value
                self.Ki = self.declare_parameter("Ki").value
                self.Kd = self.declare_parameter("Kd").value
                self.windup_limit = self.declare_parameter("windup_limit").value
                roll_effort, pitch_effort, yaw_effort = self.controlEffort()
                self.pub_roll.publish(Float64(data=roll_effort))
                self.pub_pitch.publish(Float64(data=pitch_effort))
                self.pub_yaw.publish(Float64(data=yaw_effort))
            rate.sleep()

    def calculateQuatError(self, q1, q2):
        return q1.inverse() * q2

    def controlEffort(self):
        # Calculate error values
        error_quat = self.calculateQuatError(self.body_quat, self.goal_quat)
        self.pub_error_quat.publish(Float64(data=error_quat.w))

        if error_quat.w < 0:
            error_quat = -error_quat

        curr_time = self.get_clock().now().nanoseconds / 1e9 # time in seconds
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
        self.last_integral_time = self.get_clock().now().nanoseconds / 1e9

        control_effort = proportional_effort - derivative_effort + integral_effort

        # Inertial matrix
        inertial_matrix = np.array(
            [
                [1, 0.0, 0.0],
                [0.0, 1, 0.0],
                [0.0, 0.0, 0.5],
            ]
        )

        torque = np.matmul(inertial_matrix, control_effort)
        return torque


def main(args=None):
    rclpy.init(args=args)
    pid = QuaternionPID()
    
    pid.execute()
    rclpy.spin(pid)


if __name__ == "__main__":
    main()
