#!/usr/bin/env python3

import rospy
import rostest
import unittest
import numpy as np
from object_detection_utils import find_intersection


class find_intersection_test(unittest.TestCase):
    # When the vector points straight down, the function should return a point on the plane.
    def test__VectorIntersectsPlane(self):
        vector = np.array([1, 2, 3])
        plane_z_pos = 6
        result = find_intersection(vector, plane_z_pos)
        expected_result = np.array([2, 4, 6])
        np.testing.assert_array_equal(result, expected_result)

    # When the vector is parallel to the plane, the function should return None.
    def test__VectorParallelToPlane(self):
        vector = np.array([1, 2, 0])
        plane_z_pos = 6
        result = find_intersection(vector, plane_z_pos)
        self.assertIsNone(result)

    # When the vector points away from the plane, the function should return None.
    def test__VectorPointsAwayFromPlane(self):
        vector = np.array([1, 2, -3])
        plane_z_pos = 6
        result = find_intersection(vector, plane_z_pos)
        self.assertIsNone(result)


if __name__ == "__main__":
    # rospy.init_node("find_intersection_test") - Already initialized in object_detection_utils.py
    rostest.rosrun("vision", "find_intersection_test", find_intersection_test)
