#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from auv_msgs.msg import VisionObjectArray
from std_msgs.msg import Int32MultiArray


class ObjectMapper(Node):
    def __init__(self, node_name_used):
        super().__init__(node_name_used)
        # replace with map subscriber in future
        self.map = []
        self.obj_sub = self.create_subscription(
            VisionObjectArray, "vision/object_map", self.mapUpdateCb
        )
        self.NULL_PLACEHOLDER = self.get_parameter("NULL_PLACEHOLDER").get_parameter_value()
        self.create_subscription(Int32MultiArray, "/vision/down_cam/bbox", self.callback_object_detection)

        self.delta_height = 1000
        self.delta_width = 1000
        self.distance = 1000

    def callback_object_detection(self, msg):
        bbox_x, bbox_y, image_len_x, image_len_y = msg.data

        image_x_center = image_len_x/2
        image_y_center = image_len_y/2

        self.delta_width = bbox_x-image_x_center
        self.delta_height = bbox_y-image_y_center

        self.distance = ((self.delta_height ** 2) + (self.delta_width ** 2))**0.5

    def mapUpdateCb(self, msg):
        self.map = []
        for obj in msg.array:
            new_map_obj = []
            new_map_obj.append(obj.label)
            new_map_obj.append(obj.x)
            new_map_obj.append(obj.y)
            new_map_obj.append(obj.z)
            if obj.theta_z == self.NULL_PLACEHOLDER:
                new_map_obj.append(None)
            else:
                new_map_obj.append(obj.theta_z)
            if obj.extra_field == self.NULL_PLACEHOLDER:
                new_map_obj.append(None)
            else:
                new_map_obj.append(obj.extra_field)
            self.map.append(new_map_obj)

    def getClass(self, cls=None):
        objs_with_class = []
        for obj in self.map:
            if obj[0] == cls or cls is None:
                objs_with_class.append(obj)
        return objs_with_class

    def getClosestObject(self, pos, cls=None):
        closest_object_dist = 999999
        closest_object = None
        for obj in self.map:
            if obj[0] == cls or cls is None:
                dist = math.sqrt((obj[1] - pos[0]) ** 2 + (obj[2] - pos[1]) ** 2)
                if dist < closest_object_dist:
                    closest_object = obj
                    closest_object_dist = dist
        return closest_object

    def updateObject(self, obj):
        obj = self.getClosestObject(pos=(obj[1], obj[2]), cls=obj[0])
