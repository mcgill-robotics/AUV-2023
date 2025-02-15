#!/usr/bin/env python3

import rospy
import rostest
import unittest
import cv2
import os

import sys

# Assuming test_lane_marker_measure.py is located in the tests directory
current_dir = os.path.dirname(os.path.realpath(__file__))
lane_marker_file = os.path.abspath(os.path.join(current_dir, '../../src'))
sys.path.append(lane_marker_file)
from lane_marker_measure import measure_lane_marker


# Naming convention for test functions:
# def test__testName(self):

class TestLaneMarkerMeasure(unittest.TestCase):
     @classmethod
     def setUpClass(cls):
          cls.final_line_angle_tolerance = rospy.get_param("final_line_angle_tolerance")
          cls.center_point_tolerance = rospy.get_param("center_point_tolerance")
          cls.lane_marker_folder_path = rospy.get_param("lane_marker_folder_path")
          cls.raw_image_names = []
          cls.debug_image_names = []
          cls.bboxes = []
          cls.angles_goals = []
          cls.center_point_goals = [] 

          cls.read_txt()
     
     @classmethod
     def read_txt(self):
          lane_marker_txt_file = os.path.join(self.lane_marker_folder_path, "lane_marker_data.txt")
          if not os.path.exists(lane_marker_txt_file):
            raise FileNotFoundError(f"Error: {lane_marker_txt_file} does not exist")
          with open(lane_marker_txt_file, 'r') as file:
               # Read the header line
               header = file.readline().strip().split(',')

               # Iterate over the remaining lines
               for line in file:
                    values = line.strip().split(',')
                    row = dict(zip(header, values))
                    self.raw_image_names.append(row["raw_image_file"])
                    self.debug_image_names.append(row["debug_image_file"])
                    self.bboxes.append([float(row["x"]), float(row["y"]), float(row["width"]), float(row["height"])])
                    self.angles_goals.append([float(row["angles_goal_1"]), float(row["angles_goal_2"])])
                    self.center_point_goals.append((float(row["center_point_goal_1"]), float(row["center_point_goal_2"])))

     def load_image(self, image_name):
          image_path = os.path.join(self.lane_marker_folder_path, image_name)
          if os.path.exists(image_path):
               image = cv2.imread(image_path, cv2.IMREAD_COLOR)
               return image
          return None

     def is_measure_correct(self, angles, angles_goal, center_point, center_point_goal):
          for i in range(len(angles)):
               if not (angles_goal[i] - self.final_line_angle_tolerance <= angles[i] <= angles_goal[i] + self.final_line_angle_tolerance):
                    return False
               if not (center_point_goal[i] - self.center_point_tolerance <= center_point[i] <= center_point_goal[i] + self.center_point_tolerance):
                    print(center_point[i])
                    return False
          return True
     
     def test_lane_marker_measure(self):
          for i in range(len(self.raw_image_names)):
               with self.subTest(msg=self.raw_image_names[i]):
                    image = self.load_image(self.raw_image_names[i])
                    debug_image = self.load_image(self.debug_image_names[i])
                    if image is None:
                         self.fail(f"Error: {self.raw_image_names[i]} could not be loaded.")
                    # There is an error in calling measure_lane_marker
                    angles, center_point, temp = measure_lane_marker(image, self.bboxes[i], debug_image)
                    temp_path = os.path.join(self.lane_marker_folder_path, f"test_debug_{self.raw_image_names[i]}.png")
                    cv2.imwrite(temp_path, temp)
                    if angles is None or center_point is None:
                         self.fail(f"Error: measure_lane_marker returned None for {self.raw_image_names[i]}")
                    self.assertTrue(self.is_measure_correct(angles, self.angles_goals[i], center_point, self.center_point_goals[i]))


if __name__ == "__main__":
     rostest.rosrun("vision", "test_lane_marker_measure", TestLaneMarkerMeasure)