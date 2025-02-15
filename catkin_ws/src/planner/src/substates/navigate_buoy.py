import rospy
import smach
from .utility.functions import *
import numpy as np
import quaternion
import threading
from std_msgs.msg import String


class NavigateBuoy(smach.State):
    def __init__(self, control, mapping, state):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state
        
        self.color = rospy.get_param("target_color")
        self.radius = rospy.get_param("buoy_circumnavigation_radius")
        self.offset_distance = rospy.get_param("buoy_centering_offset_distance")
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("navigate_buoy_time_limit")
        
        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("Buoy Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, ud):
        print("Starting navigate buoy.")
        self.pub_mission_display.publish("Buoy")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, rospy.get_param("nominal_depth")))
        self.control.flatten()

        buoy_object = self.mapping.getClosestObject(
            cls="Buoy", pos=(self.state.x, self.state.y)
        )
        if buoy_object is None:
            print("No buoy in object map! Failed.")
            return "failure"

        print("Centering and rotating in front of buoy.")

        if self.timeout_occurred:
            return "timeout"
        self.control.move((None, None, buoy_object[3]))

        position_auv = [self.state.x, self.state.y, self.state.z]
        position_buoy = [buoy_object[1], buoy_object[2], buoy_object[3]]

        direction = np.array(position_buoy) - np.array(position_auv)
        current_distance = np.linalg.norm(direction)

        forward_vector = np.array([1, 0, 0])
        auv_quat = self.state.quat
        auv_forward_vector = quaternion.rotate_vectors(auv_quat, forward_vector)

        rotation_quaternion = quaternion_between_vectors(
            np.array(
                [auv_forward_vector[0], auv_forward_vector[1], auv_forward_vector[2]]
            ),
            np.array([direction[0], direction[1], direction[2]]),
        )

        if self.timeout_occurred:
            return "timeout"
        self.control.rotateDelta(
            [
                rotation_quaternion.w,
                rotation_quaternion.x,
                rotation_quaternion.y,
                rotation_quaternion.z,
            ]
        )

        vector_auv_buoy = direction / current_distance * self.offset_distance
        new_position = np.array(
            [position_buoy[0], position_buoy[1], position_buoy[2]]
        ) - np.array([vector_auv_buoy[0], vector_auv_buoy[1], vector_auv_buoy[2]])
        print("Moving to new position: ", new_position)
        if self.timeout_occurred:
            return "timeout"
        self.control.move((new_position[0], new_position[1], new_position[2]))

        # wait and keep measuring just to be safe
        print("Waiting to improve measurement accuracy")
        rospy.sleep(rospy.get_param("object_observation_time"))

        if self.timeout_occurred:
            return "timeout"
        self.control.move((None, None, buoy_object[3]))

        position_auv = [self.state.x, self.state.y, self.state.z]
        position_buoy = [buoy_object[1], buoy_object[2], buoy_object[3]]

        direction = np.array(position_buoy) - np.array(position_auv)
        current_distance = np.linalg.norm(direction)

        forward_vector = np.array([1, 0, 0])
        auv_quat = self.state.quat
        auv_forward_vector = quaternion.rotate_vectors(auv_quat, forward_vector)

        rotation_quaternion = quaternion_between_vectors(
            np.array(
                [auv_forward_vector[0], auv_forward_vector[1], auv_forward_vector[2]]
            ),
            np.array([direction[0], direction[1], direction[2]]),
        )

        if self.timeout_occurred:
            return "timeout"
        self.control.rotateDelta(
            [
                rotation_quaternion.w,
                rotation_quaternion.x,
                rotation_quaternion.y,
                rotation_quaternion.z,
            ]
        )

        vector_auv_buoy = direction / current_distance * self.offset_distance
        new_position = np.array(
            [position_buoy[0], position_buoy[1], position_buoy[2]]
        ) - np.array([vector_auv_buoy[0], vector_auv_buoy[1], vector_auv_buoy[2]])
        print("Moving to new position: ", new_position)
        if self.timeout_occurred:
            return "timeout"
        self.control.move((new_position[0], new_position[1], new_position[2]))

        # Start buoy navigation
        rad = self.radius

        if self.color == "red":
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((0, -rad, 0))
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((self.offset_distance + rad, 0, 0))
            if self.timeout_occurred:
                return "timeout"
            self.control.rotateDeltaEuler((0, 0, 90))
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((2 * rad, 0, 0))
            if self.timeout_occurred:
                return "timeout"
            self.control.rotateDeltaEuler((0, 0, 90))
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((self.offset_distance + rad, 0, 0))
        else:
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((0, rad, 0))
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((self.offset_distance + rad, 0, 0))
            if self.timeout_occurred:
                return "timeout"
            self.control.rotateDeltaEuler((0, 0, -90))
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((2 * rad, 0, 0))
            if self.timeout_occurred:
                return "timeout"
            self.control.rotateDeltaEuler((0, 0, -90))
            if self.timeout_occurred:
                return "timeout"
            self.control.moveDeltaLocal((self.offset_distance + rad, 0, 0))

        self.control.freeze_pose()
        self.thread_timer.cancel()
        print("Successfully completed buoy task!")
        return "success"