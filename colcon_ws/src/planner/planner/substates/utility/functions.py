import math
import rclpy
from rclpy.clock import Clock
from tf import transformations
import numpy as np
import quaternion
from std_msgs.msg import String

def countdown(secs):
    clock = Clock()
    pub_mission_display = rclpy.create_publisher(String, "/mission_display", 1)
    end_time = clock.now().seconds_nanoseconds()[0] + secs
    while clock.now().seconds_nanoseconds()[0] < end_time and rclpy.ok():
        pub_mission_display.publish(f"T - {end_time - clock.now().seconds_nanoseconds()[0]:.3f}")


def degreesToVector(yawDegrees):
    angleRadians = yawDegrees * math.pi / 180
    x = math.cos(angleRadians)
    y = math.sin(angleRadians)
    return [x, y]


def vectorToYawDegrees(x, y):
    zero_angle_vector = np.array([1, 0])
    arg_vector = np.array([x, y])
    magnitude_arg_vector = np.linalg.norm(arg_vector)
    dot_product = np.dot(zero_angle_vector, arg_vector)
    return math.acos(dot_product / magnitude_arg_vector) * 180 / math.pi


def normalize_vector(vector2D):
    magnitude = math.sqrt(vector2D[0] ** 2 + vector2D[1] ** 2)
    if magnitude == 0:
        return (0, 0)
    normalized_x = vector2D[0] / magnitude
    normalized_y = vector2D[1] / magnitude
    return [normalized_x, normalized_y]


def dotProduct(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]


def quaternion_between_vectors(v1, v2):
    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)

    dot_product = np.dot(v1, v2)
    cross_product = np.cross(v1, v2)
    angle = np.arccos(dot_product)
    axis = cross_product / np.linalg.norm(cross_product)
    rotation_quaternion = quaternion.from_rotation_vector(axis * angle)

    return rotation_quaternion


def euler_to_quaternion(roll, pitch, yaw):
    q = transformations.quaternion_from_euler(
        math.pi * roll / 180, math.pi * pitch / 180, math.pi * yaw / 180, "rxyz"
    )
    return [q[3], q[0], q[1], q[2]]

''' 
def countdown(secs):
    pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)
    end_time = rospy.get_time() + secs
    while rospy.get_time() < end_time:
        pub_mission_display.publish(str(end_time - rospy.get_time()))
'''
