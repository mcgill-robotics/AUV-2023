#!/usr/bin/env python3

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node
import actionlib
from geometry_msgs.msg import Pose, Vector3, Vector3Stamped, Wrench
from std_msgs.msg import Float64, Bool, Header
from auv_msgs.msg import (
    EffortAction,
    EffortGoal,
    StateQuaternionAction,
    StateQuaternionGoal,
    ThrusterMicroseconds,
)
from controls.controls.clients import SimpleClient
from actionlib_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
from .functions import *
import numpy as np
import quaternion
import math


# predefined bools so we don't have to write these out everytime we want to get a new goal

do_displace = Bool(True)
do_not_displace = Bool(False)
is_local = Bool(True)
is_not_local = Bool(False)

"""
Helper class for the planner. Takes in simple commands, converts them to 
goals and sends them to the control servers.
"""


class Controller(Node):
    def __init__(self, header_time, node_name_used):
        super().__init__(node_name_used)
        self.get_logger().info("starting controller")
        self.header_time = header_time
        self.clock = Clock()
        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.orientation = None

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer)
        self.tf_header = Header(frame_id="world_rotation")

        self.sub = self.create_subscription(Pose, "/state/pose", self.set_position)
        self.sub_theta_x = self.create_subscription(Float64, "/state/pose", self.set_theta_x)
        self.sub_theta_y = self.create_subscription(Float64, "/state/theta/y", self.set_theta_y)
        self.sub_theta_z = self.create_subscription(Float64, "/state/theta/z", self.set_theta_z)

        self.pub_x_enable = self.create_publisher(
            Bool, "/controls/pid/x/enable", 1
        )
        self.pub_y_enable = self.create_publisher(
            Bool, "/controls/pid/y/enable", 1
        )
        self.pub_z_enable = self.create_publisher(
            "/controls/pid/z/enable", Bool, 1
        )
        self.pub_quat_enable = self.create_publisher(
            "/controls/pid/quat/enable", Bool, 1
        )

        # for killing
        self.pub_surge = self.create_publisher(Float64, "/controls/force/surge", 1)
        self.pub_sway = self.create_publisher(Float64, "/controls/force/sway", 1)
        self.pub_heave = self.create_publisher(Float64, "/controls/force/heave", 1)
        self.pub_roll = self.create_publisher(Float64, "/controls/torque/roll", 1)
        self.pub_pitch = self.create_publisher(
           Float64, "/controls/torque/pitch", 1
        )
        self.pub_yaw = self.create_publisher(Float64, "/controls/torque/yaw", 1)
        self.pub_effort =self.create_publisher(Wrench, "/controls/effort", 1)
        self.pub_global_x = self.create_publisher(
            Float64, "/controls/force/global/x", 1
        )
        self.pub_global_y = self.create_publisher(
            Float64, "/controls/force/global/y", 1
        )
        self.pub_global_z = self.create_publisher(
            Float64, "/controls/force/global/z", 1
        )

        # Create publishers for the dropper topic and the claw state topic
        self.claw_state_pub = self.create_publisher(
            Bool, "/actuators/grabber/close", 1
        )

        self.pwm_pub = self.create_publisher(
            ThrusterMicroseconds, "/propulsion/microseconds", 1
        )

        self.clients = []

        self.EffortClient = SimpleClient(
            "/controls/server/effort", EffortAction, "EffortClientNode"
        )
        self.clients.append(self.EffortClient)
        self.get_logger().info("Waiting for EffortServer to come online...")

        self.StateQuaternionStateClient = SimpleClient(
            "/controls/server/state", StateQuaternionAction, "StateQuaternionStateClientNode"
        )
        self.clients.append(self.StateQuaternionStateClient)
        self.get_logger().info("Waiting for StateQuaternionStateServer to come online...")

        self.get_logger().info("Controller waiting to receive state information...")

        while (
            None
            in [
                self.x,
                self.y,
                self.z,
                self.theta_x,
                self.theta_y,
                self.theta_z,
                self.orientation,
            ]
            and rclpy.ok()
        ):
            debug_str = "Missing state information for "
            for state_axis, state_axis_name in [
                (self.x, "x"),
                (self.y, "y"),
                (self.z, "z"),
                (self.theta_x, "theta x"),
                (self.theta_y, "theta y"),
                (self.theta_z, "theta z"),
                (self.orientation, "quat."),
            ]:
                if state_axis is None:
                    debug_str += state_axis_name + ", "
            self.get_logger().info(debug_str)
            rate = self.create_timer(1)
            rate.sleep()
            

        self.get_logger().info("All state information received, controller is active.")

    def set_position(self, data):
        self.x = data.position.x
        self.y = data.position.y
        self.z = data.position.z
        self.orientation = data.orientation

    def set_theta_x(self, data):
        self.theta_x = data.data

    def set_theta_y(self, data):
        self.theta_y = data.data

    def set_theta_z(self, data):
        self.theta_z = data.data

    def transformLocalToGlobal(self, lx, ly, lz):
        """
        Performs a coordinate transformation from the auv body frame
        to the world frame.
        """
        trans = self.tf_buffer.lookup_transform(
            "world_rotation", "auv_rotation", self.header_time
        )
        offset_local = Vector3(lx, ly, lz)
        self.tf_header.stamp = self.header_time
        offset_local_stmp = Vector3Stamped(header=self.tf_header, vector=offset_local)
        offset_global = tf2_geometry_msgs.do_transform_vector3(offset_local_stmp, trans)
        return (
            float(offset_global.vector.x),
            float(offset_global.vector.y),
            float(offset_global.vector.z),
        )

    # method to easily get goal object
    def get_effort_goal(self, dofs):
        surge, sway, heave, roll, pitch, yaw = dofs

        goal = EffortGoal()
        goal.effort.force.x = 0 if surge is None else surge
        goal.do_surge = Bool(False) if surge is None else Bool(True)

        goal.effort.force.y = 0 if sway is None else sway
        goal.do_sway = Bool(False) if sway is None else Bool(True)

        goal.effort.force.z = 0 if heave is None else heave
        goal.do_heave = Bool(False) if heave is None else Bool(True)

        goal.effort.torque.x = 0 if roll is None else roll
        goal.do_roll = Bool(False) if roll is None else Bool(True)

        goal.effort.torque.y = 0 if pitch is None else pitch
        goal.do_pitch = Bool(False) if pitch is None else Bool(True)

        goal.effort.torque.z = 0 if yaw is None else yaw
        goal.do_yaw = Bool(False) if yaw is None else Bool(True)

        return goal

    # method to easily get goal object
    def get_state_goal(self, state, displace, local=is_not_local):
        x, y, z, tw, tx, ty, tz = state
        goal = StateQuaternionGoal()

        goal.displace = displace
        goal.local = local

        goal.pose.position.x = 0 if x is None else x
        goal.do_x = Bool(False) if x is None else Bool(True)

        goal.pose.position.y = 0 if y is None else y
        goal.do_y = Bool(False) if y is None else Bool(True)

        goal.pose.position.z = 0 if z is None else z
        goal.do_z = Bool(False) if z is None else Bool(True)

        goal.pose.orientation.w = 1 if tw is None else tw
        goal.pose.orientation.x = 0 if tx is None else tx
        goal.pose.orientation.y = 0 if ty is None else ty
        goal.pose.orientation.z = 0 if tz is None else tz
        goal.do_quaternion = Bool(False) if tz is None else Bool(True)

        return goal

    # preempt the current action done in the server
    def preemptCurrentAction(self):
        for client in self.clients:
            if client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                client.cancel_goal()

    # rotate to this rotation (quaternion)
    def rotate(self, ang):
        if any(x is None for x in ang) and any(x is not None for x in ang):
            raise ValueError(
                "Invalid rotate goal: quaternion cannot have a combination of None and valid values. Goal received: {}".format(
                    ang
                )
            )
        w, x, y, z = ang
        goal_state = self.get_state_goal(
            [None, None, None, w, x, y, z], do_not_displace
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    # rotate to this rotation (euler)
    def rotateEuler(self, ang):
        x, y, z = ang
        if x is None:
            x = self.theta_x
        if y is None:
            y = self.theta_y
        if z is None:
            z = self.theta_z
        self.rotate(euler_to_quaternion(x, y, z))

    def state(self, pos, ang):
        x, y, z = pos
        if any(x is None for x in ang) and any(x is not None for x in ang):
            raise ValueError(
                "Invalid state goal: quaternion cannot have a combination of None and valid values. Goal received: {}".format(
                    ang
                )
            )
        w, wx, wy, wz = ang
        goal_state = self.get_state_goal([x, y, z, w, wx, wy, wz], do_not_displace)
        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    def stateDelta(self, pos, ang):
        x, y, z = pos
        if any(x is None for x in ang) and any(x is not None for x in ang):
            raise ValueError(
                "Invalid stateDelta goal: quaternion cannot have a combination of None and valid values. Goal received: {}".format(
                    ang
                )
            )
        w, wx, wy, wz = ang
        goal_state = self.get_state_goal([x, y, z, w, wx, wy, wz], do_displace)
        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    def stateEuler(self, pos, ang):
        wx, wy, wz = ang
        if wx is None:
            wx = self.theta_x
        if wy is None:
            wy = self.theta_y
        if wz is None:
            wz = self.theta_z
        self.state(pos, euler_to_quaternion(wx, wy, wz))

    def stateDeltaEuler(self, pos, ang):
        wx, wy, wz = ang
        if wx is None:
            wx = 0
        if wy is None:
            wy = 0
        if wz is None:
            wz = 0
        self.stateDelta(pos, euler_to_quaternion(wx, wy, wz))

    # move to setpoint
    def move(self, pos, face_destination=False):
        x, y, z = pos

        goal_state = self.get_state_goal(
            [x, y, z, None, None, None, None], do_not_displace
        )

        x = self.x if x is None else x
        y = self.y if y is None else y
        if face_destination and math.sqrt(x**2 + y**2) > 0.5:
            yaw_towards_destination = vectorToYawDegrees(x - self.x, y - self.y)
            self.rotateEuler((0, 0, yaw_towards_destination))

        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    # move by this amount in world space
    def moveDelta(self, delta, face_destination=False):
        x, y, z = delta

        goal_state = self.get_state_goal([x, y, z, None, None, None, None], do_displace)

        x = 0 if x is None else x
        y = 0 if y is None else y
        if face_destination and math.sqrt(x**2 + y**2) > 0.5:
            yaw_towards_destination = vectorToYawDegrees(x, y)
            self.rotateEuler((0, 0, yaw_towards_destination))

        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    # rotate by this amount (quaternion)
    def rotateDelta(self, delta):
        if any(x is None for x in delta) and any(x is not None for x in delta):
            raise ValueError(
                "Invalid rotateDelta goal: quaternion cannot have a combination of None and valid values. Goal received: {}".format(
                    delta
                )
            )
        w, x, y, z = delta
        goal_state = self.get_state_goal([None, None, None, w, x, y, z], do_displace)

        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    # rotate by this amount (euler)
    def rotateDeltaEuler(self, delta):
        x, y, z = delta
        if x is None:
            x = 0
        if y is None:
            y = 0
        if z is None:
            z = 0
        self.rotateDelta(euler_to_quaternion(x, y, z))

    # move by this amount in local space (i.e. z is always heave)
    def moveDeltaLocal(self, delta, face_destination=False):
        x, y, z = delta
        goal_state = self.get_state_goal(
            [x, y, z, None, None, None, None], do_displace, local=is_local
        )

        if x is None:
            x = 0
        if y is None:
            y = 0
        if face_destination and math.sqrt(x**2 + y**2) > 0.5:
            yaw_towards_destination = vectorToYawDegrees(x, y)
            self.rotateDeltaEuler((0, 0, yaw_towards_destination))

        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    # set torque
    def torque(self, vel):
        x, y, z = vel
        goal = self.get_effort_goal([None, None, None, x, y, z])
        self.EffortClient.send_goal(goal)

    # set positional effort in local space (i.e. z is always heave)
    def forceLocal(self, vel):
        x, y = vel
        goal = self.get_effort_goal([x, y, None, None, None, None])
        self.EffortClient.send_goal(goal)

    # stop all thrusters
    def kill(self):
        self.preemptCurrentAction()
        goal = self.get_effort_goal([0, 0, 0, 0, 0, 0])
        self.EffortClient.send_goal(goal)
        self.pub_x_enable.publish(Bool(False))
        self.pub_y_enable.publish(Bool(False))
        self.pub_z_enable.publish(Bool(False))
        self.pub_quat_enable.publish(Bool(False))

        start = self.clock.now().seconds_nanoseconds()[0]
        while self.clock.now().seconds_nanoseconds()[0] - start < 5:
            self.pub_surge.publish(0)
            self.pub_sway.publish(0)
            self.pub_heave.publish(0)
            self.pub_roll.publish(0)
            self.pub_pitch.publish(0)
            self.pub_yaw.publish(0)
            self.pub_global_x.publish(0)
            self.pub_global_y.publish(0)
            self.pub_global_z.publish(0)

            zero_wrench = Wrench()
            zero_wrench.force.x = 0
            zero_wrench.force.y = 0
            zero_wrench.force.z = 0
            zero_wrench.torque.x = 0
            zero_wrench.torque.y = 0
            zero_wrench.torque.z = 0
            self.pub_effort.publish(zero_wrench)

            msg = ThrusterMicroseconds([1500] * 8)
            self.pwm_pub.publish(msg)

    def freeze_pose(self):
        goal = self.get_state_goal(
            [
                self.x,
                self.y,
                self.z,
                self.orientation.w,
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
            ],
            do_not_displace,
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    def freeze_position(self):
        goal = self.get_state_goal(
            [self.x, self.y, self.z, None, None, None, None], do_not_displace
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    def freeze_rotation(self):
        goal = self.get_state_goal(
            [
                None,
                None,
                None,
                self.orientation.w,
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
            ],
            do_not_displace,
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    def flatten(self):
        orientation = np.quaternion(
            self.orientation.w,
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
        )
        v = orientation * np.quaternion(0, 1, 0, 0) * orientation.conjugate()
        v = np.array([v.x, v.y])
        v = v / np.linalg.norm(v)
        forward = np.array((1, 0))
        angle = math.acos(v.dot(forward))
        if v[0] < 0:
            angle *= -1
        final = np.quaternion(math.cos(angle / 2), 0, 0, math.sin(angle / 2))

        goal = self.get_state_goal(
            [None, None, None, final.w, final.x, final.y, final.z], do_not_displace
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    def open_claw(self):

        # Set the claw state to True (open)
        self.claw_state_pub.publish(Bool(True))

    def close_claw(self):
        # Set the claw state to False (close)
        self.claw_state_pub.publish(Bool(False))
