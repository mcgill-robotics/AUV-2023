#!/usr/bin/env python3

import rclpy
import numpy as np

from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge

from common_utils import crop_to_bbox
from point_cloud import get_xyz_image

from auv_msgs.msg import VisionObjectArray