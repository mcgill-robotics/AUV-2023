#!/usr/bin/env python3

import rospy
import rostest
import unittest
import math
import numpy as np
from object_detection_utils import eulerToVectorDownCam


class euler_to_vector_test(unittest.TestCase):
    # When both angles are 0 degrees, the function should return a vector pointing straight down.
    def test__ZeroDegrees(self):
        vec = eulerToVectorDownCam(0, 0)
        self.assertTrue(np.allclose(vec, np.array([0, 0, -1])))

    # When both angles are 45 degrees, the function should return a vector pointing diagonally.
    def test__45Degrees(self):
        vec = eulerToVectorDownCam(45, 45)
        expected_vec = np.array([-1, 1, -1]) / math.sqrt(3)
        self.assertTrue(np.allclose(vec, expected_vec))

    # When both angles are 90 degrees, the function should return a vector pointing straight up.
    def test__90Degrees(self):
        vec = eulerToVectorDownCam(90, 90)
        expected_vec = np.array([-1, 1, 0]) / math.sqrt(2)
        self.assertTrue(np.allclose(vec, expected_vec))

    # When both angles are -45 degrees, the function should return a vector pointing diagonally in the opposite direction.
    def test__NegativeDegrees(self):
        vec = eulerToVectorDownCam(-45, -45)
        expected_vec = np.array([1, -1, -1]) / math.sqrt(3)
        self.assertTrue(np.allclose(vec, expected_vec))


if __name__ == "__main__":
    # rospy.init_node("euler_to_vector_test") - Already initialized in object_detection_utils.py
    rostest.rosrun("vision", "euler_to_vector_test", euler_to_vector_test)
