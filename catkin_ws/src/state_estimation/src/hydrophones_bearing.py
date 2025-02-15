#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import PingerBearing, PingerTimeDifference
from geometry_msgs.msg import Pose
from std_msgs.msg import Int32


def calculate_time_measurements(delta_time):
    distance = delta_time * c
    return distance

def solve_bearing_vector(distance, is_three_hydrophones):
    position = np.array([hydrophones_dx, hydrophones_dy, 0] if is_three_hydrophones else [hydrophones_dx, hydrophones_dy, hydrophones_dz])
    with np.errstate(divide='ignore', invalid='ignore'):
        bearing_vector = np.where(position != 0, distance / position, 0)
    return bearing_vector


def cb_hydrophones_time_difference(msg):
    if not is_active or msg.frequency == 0:
        return
    
    # Convert times received in 10e-7 to seconds
    absolute_times = np.array(msg.times) * time_unit
    
    # Calculate time differences between hydrophone 0 and others.
    dt_hydrophones = absolute_times - absolute_times[0]
    # Only take time differences between microphone 0 and others.
    dt_hydrophones = dt_hydrophones[1:]
    
    is_three_hydrophones = False
    if len(dt_hydrophones) == 2:
        dt_hydrophones = np.append(dt_hydrophones, 0)
        is_three_hydrophones = True
    
    measurements = calculate_time_measurements(dt_hydrophones)
    bearing_vector_local = solve_bearing_vector(measurements, is_three_hydrophones)
    bearing_vector_global = quaternion.rotate_vectors(
        auv_rotation,
        np.array(bearing_vector_local)
    )

    if is_three_hydrophones:
        bearing_vector_global[2] = 0

    PingerBearing_msg = PingerBearing()
    PingerBearing_msg.frequency = msg.frequency
    PingerBearing_msg.pinger_bearing.x = bearing_vector_global[0]
    PingerBearing_msg.pinger_bearing.y = bearing_vector_global[1]
    PingerBearing_msg.pinger_bearing.z = bearing_vector_global[2]
    PingerBearing_msg.state_x = auv_position[0]
    PingerBearing_msg.state_y = auv_position[1]
    PingerBearing_msg.state_z = auv_position[2]

    pub_pinger_bearing.publish(PingerBearing_msg)
   
def cb_hydrophones_status(msg):
    global is_active
    is_active = msg.data

def cb_pose(msg):
    global auv_position, auv_rotation
    auv_position[0] = msg.position.x
    auv_position[1] = msg.position.y
    auv_position[2] = msg.position.z
    auv_rotation = np.quaternion(
        msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
    )


if __name__ == "__main__":
    rospy.init_node("hydrophones_bearing")

    hydrophones_dx = rospy.get_param("hydrophones_dx")
    hydrophones_dy = rospy.get_param("hydrophones_dy")
    hydrophones_dz = rospy.get_param("hydrophones_dz")

    time_unit = rospy.get_param("hydrophones_time_unit")

    auv_position = [0, 0, 0]
    auv_rotation = np.quaternion(1, 0, 0, 0)
    is_active = False

    rospy.Subscriber (
        "/sensors/hydrophones/status",
        Int32,
        cb_hydrophones_status
    )

    rospy.Subscriber(
        "/sensors/hydrophones/pinger_time_difference",
        PingerTimeDifference,
        cb_hydrophones_time_difference,
    )

    rospy.Subscriber("/state/pose", Pose, cb_pose)

    pub_pinger_bearing = rospy.Publisher(
        "/sensors/hydrophones/pinger_bearing", PingerBearing, queue_size=1
    )

    # Speed of sound in water
    c = 1480    

    rospy.spin()
