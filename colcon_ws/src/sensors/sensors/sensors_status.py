#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
import rclpy.logging
import numpy as np
import quaternion

from auv_msgs.mag import PingerTimeDifference
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

class Sensor:
    """
    Class representation of a Sensor.
    """

    def __init__(
        self, 
        sensor_name, 
        time_before_considered_inactive, 
        sensor_warning_interval
    ): 
        self.sensor_name = sensor_name
        self.time_before_considered_inactive = time_before_considered_inactive
        self.sensor_warning_interval = sensor_warning_interval

        # Initialize the clock and logger
        self.clock = Clock()
        self.logger = rclpy.logging.get_logger()

        # initialize a sensor as "inactive"
        self.last_unique_reading_time = -1 * self.time_before_considered_inactive
        self.current_reading = None
        self.last_reading = None
        self.is_active = False
        self.last_error_message_time = self.get_current_time()
        self.last_inactive_message_time = self.get_current_time()
    
    def get_current_time(self):
        return self.clock.now()
    
    def get_logger(self):
        return self.logger
        
    def update_last_reading(self):
        if self.current_reading != self.last_reading:
            self.last_unique_reading_time = self.get_current_time()
            self.last_reading = self.current_reading

    def get_is_active(self):
        if (
            not self.has_valid_data()
            or self.get_current_time() - self.last_unique_reading_time
            > self.time_before_considered_inactive
        ):
            if (
                self.get_current_time() - self.last_inactive_message_time
                > self.sensor_warning_interval
            ):
                self.last_inactive_message_time = self.get_current_time()
                self.get_logger().warn("{} is inactive.".format(self.sensor_name))
            self.is_active = False
            return 0
        else:
            if not self.is_active:
                # rospy.loginfo("{} has become active.".format(self.sensor_name))
                i = 0
            self.is_active = True
            return 1

    def has_valid_data(self):
        raise NotImplementedError("Subclass must implement abstract method")
    
class DepthSensor(Sensor):
    """
    self.current_reading == z
    z: float
    """

    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        node_functions_cb: function
    ): 
        super().__init__(
            "Depth Sensor", 
            time_before_considered_inactive, 
            sensor_warning_interval
        )
        node_functions_cb(self)

    def depth_cb(self, msg):
        self.current_reading = msg.data
        self.update_last_reading()

    def has_valid_data(self): 
         return self.current_reading is not None


class IMU(Sensor):
    """
    self.current_reading == [quaternion, angular_velocity]
    quaternion: np.quaternion(w, x, y, z)
    angular_velocity = np.array([x, y, z])
    """

    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        node_functions_cb: function
    ):
        super().__init__("IMU", time_before_considered_inactive, sensor_warning_interval)
        self.current_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]
        self.last_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]
        node_functions_cb(self)

    def imu_cb(self, msg):
        q = msg.orientation
        ang_vel = msg.angular_velocity
        self.current_reading[0] = np.quaternion(q.w, q.x, q.y, q.z)
        self.current_reading[1] = [ang_vel.x, ang_vel.y, ang_vel.z]
        self.update_last_reading()

    def update_last_reading(self):
        if self.current_reading != self.last_reading:
            self.last_unique_reading_time = self.get_current_time()
            self.last_reading[0] = self.current_reading[0]  # np.quaternion
            self.last_reading[1] = self.current_reading[1][:]  # gyro (list) -> deepcopy

    def has_valid_data(self):
        return (
            self.current_reading[0] is not None and self.current_reading[1] is not None
        )
        
class FrontCameraIMU(Sensor):
    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        node_functions_cb: function
    ):
        super().__init__("Front Camera IMU", time_before_considered_inactive, sensor_warning_interval)
        self.current_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]
        self.last_reading = [np.quaternion(1, 0, 0, 0), [0, 0, 0]]

    def front_camera_imu_cb(self, msg):
        self.current_reading[0] = np.quaternion(
            msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z
        )
        self.current_reading[1] = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        )
        self.update_last_reading()

    def update_last_reading(self):
        if self.current_reading != self.last_reading:
            self.last_unique_reading_time = self.get_current_time()
            self.last_reading[0] = self.current_reading[0]
            self.last_reading[1] = self.current_reading[1][:]

    def has_valid_data(self):
        return (
            self.current_reading[0] is not None and self.current_reading[1] is not None
        )

class DVL(Sensor):
    """
    self.current_reading == [x, y, z, roll, pitch, yaw]
    x, y, z, roll, pitch, yaw: float
    """

    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        node_functions_cb: function
    ):
        super().__init__("DVL", time_before_considered_inactive, sensor_warning_interval)
        
        self.current_reading = [None, None, None]
        self.last_reading = [None, None, None]

        node_functions_cb(self)

    def twist_cb(self, msg):
        self.current_reading = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z]
        self.update_last_reading()

    def update_last_reading(self):
        if self.current_reading != self.last_reading:
            self.last_unique_reading_time = self.get_current_time()
            self.last_reading = self.current_reading[:]

    def has_valid_data(self):
        return None not in self.current_reading

class FrontCameraImage(Sensor):
    """
    self.current_reading == frame_id
    frame_id: string
    """

    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        node_functions_cb: function
    ):
        super().__init__("Front Camera Image", time_before_considered_inactive, sensor_warning_interval)

        node_functions_cb(self)
    
    def front_camera_cb(self, msg):
        time = msg.header.stamp
        self.current_reading = time.secs * 1e9 + time.nsecs
        self.update_last_reading()

    def has_valid_data(self):
        return self.current_reading is not None

class DownCamera(Sensor):
    """
    self.current_reading == frame_id
    frame_id: string
    """

    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        node_functions_cb: function
    ):
        super().__init__("Down Camera", time_before_considered_inactive, sensor_warning_interval)

        node_functions_cb(self)
    
    def down_camera_cb(self, msg):
        time = msg.header.stamp
        self.current_reading = time.secs * 1e9 + time.nsecs
        self.update_last_reading()

    def has_valid_data(self):
        return self.current_reading is not None

# TODO: Add Parameters through parameter of the constructor
class Hydrophones(Sensor):
    """
    Class representation of the four hydrophone sensors as a group
    """
    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        hydrophones_time_difference_tolerance,
        node_functions_cb: function
    ):
        super().__init__("Hydrophones", time_before_considered_inactive, sensor_warning_interval)

        self.current_reading = {}
        self.last_reading = {}
        self.time_tolerance = hydrophones_time_difference_tolerance

        node_functions_cb(self)
    
    def hydrophones_cb(self, msg):
        times = msg.times
        if (max(times) - min(times) <= self.time_tolerance):
            self.current_reading[msg.frequency] = times
            self.update_last_reading(msg.frequency)

    def update_last_reading(self, frequency):
        if self.current_reading.get(frequency, []) != self.last_reading.get(frequency, []):
            self.last_unique_reading_time = self.get_current_time()
            self.last_reading[frequency] = self.current_reading.get(frequency, [])[:]

    def has_valid_data(self):
        return len(self.current_reading.keys()) > 0

class Actuator(Sensor):
    """
    WAITING FOR ELECTRICAL FOR:
         - topic name
         - message type
    self.current_reading = position
    status: (int (0 or 1) or bool) - not yet decided
    position: (int or float) - not yet decided
    """
    def __init__(
        self, 
        time_before_considered_inactive, 
        sensor_warning_interval,
        node_functions_cb: function
    ):
        super().__init__("Actuator", time_before_considered_inactive, sensor_warning_interval)
        self.status = False 
        node_functions_cb(self)

    def actuator_cb(self, msg):
        # self.status = msg.status
        # self.current_reading = msg.position
        self.update_last_reading()
        
    def has_valid_data(self):
        return self.status and self.current_reading is not None


class SensorStatus(Node):
    def __init__(self):
        self.declare_parameter("time_before_considered_inactive")
        self.declare_parameter("sensor_warning_interval")
        self.declare_parameter("hydrophones_time_difference_tolerance")
        self.declare_parameter("sensor_status_update_rate")

        time_before_considered_inactive = self.get_parameter("time_before_considered_inactive")
        sensor_warning_interval = self.get_parameter("sensor_warning_interval")
        """
        Sensor Objects:
        The sensor classes will instantiate an abstract of a sensor.

        Pass all the required ROS parameters to the constructor of the sensor object.
        Invoke callbacks on the sensor instance to "retrieve" data from the processed data.

        To access Node Class methods:
        Create a lambda function that passes the node calls into the constructor of the objects.
        """
        self.depth_sensor = DepthSensor(
            time_before_considered_inactive,
            sensor_warning_interval,
            lambda sensor_object: (
                self.create_subscription(
                    Float64,
                    "/sensors/depth/z",
                    sensor_object.depth_cb()
                )
            )
        )
        self.imu = IMU(
            time_before_considered_inactive,
            sensor_warning_interval,
            lambda sensor_object: (
                self.create_subscription(
                    Imu, 
                    "/sensors/imu/data", 
                    sensor_object.imu_cb()
                )
            )
        )
        self.imu_front_camera = FrontCameraIMU(
            time_before_considered_inactive,
            sensor_warning_interval,
            lambda sensor_object: (
                self.create_subscription(
                    Imu,
                    "/zed/zed_node/imu/data",
                    sensor_object.front_camera_imu_cb()
                )
            )
        )
        self.dvl = DVL(
            time_before_considered_inactive,
            sensor_warning_interval,
            lambda sensor_object: (
                self.create_subscription(
                    TwistWithCovarianceStamped,
                    "/sensors/dvl/twist",
                    sensor_object.twist_cb()
                )
            )
        )
        self.front_camera_image = FrontCameraImage(
            time_before_considered_inactive,
            sensor_warning_interval,
            lambda sensor_object: (
                self.create_subscription(
                    Image,
                    "/zed/zed_node/stereo/image_rect_color",
                    sensor_object.front_camera_cb()
                )
            )
        )
        self.down_camera = DownCamera(
            time_before_considered_inactive,
            sensor_warning_interval,
            lambda sensor_object: (
                self.create_subscription(
                    
                )
            )
        )
        self.hydrophones = Hydrophones(
            time_before_considered_inactive,
            sensor_warning_interval
        )
        self.actuator = Actuator(
            time_before_considered_inactive,
            sensor_warning_interval
        )

        self.pub_depth_sensor_status = self.create_publisher(
            Int32, '/sensors/depth/status', 1
        )
        self.pub_imu_sensor_status = self.create_publisher(
            Int32, '/sensors/imu/status', 1
        )
        self.pub_imu_front_camera_sensor_status = self.create_publisher(
            Int32, '/sensors/imu_front_camera/status', 1
        )
        self.pub_dvl_sensor_status = self.create_publisher(
            Int32, '/sensors/dvl/status', 1
        )
        self.pub_front_camera_sensor_status = self.create_publisher(
            Int32, '/sensors/front_camera/status', 1
        )
        self.pub_down_camera_sensor_status = self.create_publisher(
            Int32, '/sensors/down_camera/status', 1
        )
        self.pub_hydrophones_sensor_status = self.create_publisher(
            Int32, '/sensors/hydrophones/status', 1
        )
        self.pub_actuator_sensor_status = self.create_publisher(
            Int32, '/sensors/actuator/status', 1
        )
        # From Albert: Not sure which data type this parameter is, assumed as Float64
        self.sensor_status_update_rate = self.declare_parameter('sensor_status_update_rate', Float64)

        self.timer = self.create_timer(
            1.0 / self.sensor_status_update_rate,
            self.update_state
        )

    # TODO: Document what update_state does
    def update_state(self):
        self.pub_depth_sensor_status.publish(self.depth_sensor.get_is_active())
        self.pub_imu_sensor_status.publish(self.imu.get_is_active())
        self.pub_imu_front_camera_sensor_status.publish(self.imu_front_camera.get_is_active())
        self.pub_dvl_sensor_status.publish(self.dvl.get_is_active())
        self.pub_front_camera_sensor_status.publish(self.front_camera_image.get_is_active())
        self.pub_down_camera_sensor_status.publish(self.down_camera.get_is_active())
        self.pub_hydrophones_sensor_status.publish(self.hydrophones.get_is_active())
        self.pub_actuator_sensor_status.publish(self.actuator.get_is_active())

        return
        
def main():
    rclpy.init_node('sensor_status')

    sensor_status = SensorStatus()

    rclpy.spin(sensor_status)
    sensor_status.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()