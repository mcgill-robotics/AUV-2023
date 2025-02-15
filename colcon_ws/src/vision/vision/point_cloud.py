import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_system_default

from rclpy.node import Node

from cv_bridge import CvBridge

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, Image, CameraInfo, PointField
from sensor_msgs import point_cloud2

class PointCloud(Node):
    def __init__(self):
        self.bridge = CvBridge()

        # Initialize important camera variables
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.width = None
        self.height = None

        # Initialize images to None
        self.x_over_z_map = None
        self.y_over_z_map = None
        self.convert_map = None
        self.rgb = None
        self.depth = None

        # Get all parameters needed
        self.declare_parameter("depth_map_scale_factor")
        self.DEPTH_SCALE_FACTOR = self.get_parameter("depth_map_scale_factor")

        self.declare_parameter("front_cam_x_offset")
        self.declare_parameter("front_cam_y_offset")
        self.declare_parameter("front_cam_z_offset")

        # TODO: Set up Publishers and Subscribers
        # ...

        self.point_cloud_pub = self.create_publisher(PointCloud2, "vision/front_cam/point_cloud_raw", 3)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/vision/front_cam/camera_info",
            self.camera_info_callback,
            qos_profile_system_default
        )

    def rbg_callback(self, msg):
        temp = self.bridge.imgmsg_to_cv2(msg)
        self.rgb = temp / 255

    def depth_callback(self, msg):
        temp = self.bridge.imgmsg_to_cv2(msg)
        self.depth = temp / self.DEPTH_SCALE_FACTOR

    def camera_info_callback(self, msg):
        # if(y_over_z_map is not None): return
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

        self.width = msg.width
        self.height = msg.height

        u_map = np.tile(np.arange(self.width), (self.height, 1)) + 1
        v_map = np.tile(np.arange(self.height), (self.width, 1)).T + 1

        self.x_over_z_map = (self.cx - u_map) / self.fx
        self.y_over_z_map = (self.cy - v_map) / self.fy

    def convert_from_uvd(self, width, height):
        if self.y_over_z_map is not None:
            time = self.get_clock().now()
            xyz_rgb_image = self.get_xyz_rgb_image(
                self.rgb, self.depth, width, height, self.x_over_z_map, self.y_over_z_map
            )

            xyz_rgb_image = xyz_rgb_image.reshape((width * height, 6))
            xyz_rgb_image = xyz_rgb_image.astype(np.float32)
            fields = [
                PointField("x", 0, PointField.FLOAT32, 1),
                PointField("y", 4, PointField.FLOAT32, 1),
                PointField("z", 8, PointField.FLOAT32, 1),
                PointField("r", 12, PointField.FLOAT32, 1),
                PointField("g", 16, PointField.FLOAT32, 1),
                PointField("b", 20, PointField.FLOAT32, 1),
            ]

            header = Header()
            header.stamp = time
            header.frame_id = "auv_base"
            pub_msg = point_cloud2.create_cloud(
                header=header, fields=fields, points=xyz_rgb_image
            )
            return pub_msg

    def get_point_cloud_image(
        self, bridge, color, z_map, width, height, x_over_z_map, y_over_z_map
    ):
        if y_over_z_map is not None:
            xyz_rgb_image = self.get_xyz_rgb_image(
                color, z_map, width, height, x_over_z_map, y_over_z_map
            )
            point_cloud_image = bridge.cv2_to_imgmsg(np.float32(xyz_rgb_image[:, :, :3]))
            return point_cloud_image


    def get_xyz_rgb_image(self, color, z_map, width, height, x_over_z_map, y_over_z_map):
        if y_over_z_map is not None:
            xyz_rgb_image = np.zeros((height, width, 6))
            xyz_rgb_image[:, :, 3:6] = color[:, :, 0:3]

            x_map = x_over_z_map * z_map
            y_map = y_over_z_map * z_map

            xyz_rgb_image[:, :, 0] = z_map + self.get_parameter_or("front_cam_x_offset", 0)
            xyz_rgb_image[:, :, 1] = x_map + self.get_parameter_or("front_cam_y_offset", 0)
            xyz_rgb_image[:, :, 2] = y_map + self.get_parameter_or("front_cam_z_offset", 0)

            return xyz_rgb_image


    def get_xyz_image(self, z_map, width, height, x_over_z_map, y_over_z_map):
        if y_over_z_map is not None:
            xyz_image = np.zeros((height, width, 3))

            x_map = x_over_z_map * z_map
            y_map = y_over_z_map * z_map

            xyz_image[:, :, 0] = z_map + self.get_parameter_or("front_cam_x_offset", 0)
            xyz_image[:, :, 1] = x_map + self.get_parameter_or("front_cam_y_offset", 0)
            xyz_image[:, :, 2] = y_map + self.get_parameter_or("front_cam_z_offset", 0)

            return xyz_image

def main(args=None):
    rclpy.init(args=args)

    try:
        point_cloud_node = PointCloud()
        rclpy.spin(point_cloud_node)

    except KeyboardInterrupt:
        point_cloud_node.get_logger().info("Interrupted... terminating...")

    except Exception as exc:
        point_cloud_node.get_logger().error(f"An error occured with exception: {exc}")

    finally:
        point_cloud_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()