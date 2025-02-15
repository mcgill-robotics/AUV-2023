#!/usr/bin/env python3

import rospy
import numpy as np

from auv_msgs.msg import UnityState, PingerTimeDifference
from geometry_msgs.msg import Quaternion, Vector3, TwistWithCovarianceStamped, Pose, TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf import transformations
import quaternion
from tf2_ros import TransformBroadcaster
from robot_localization.srv import SetPose

DEG_PER_RAD = 180 / np.pi
NUMBER_OF_PINGERS = 4

def reset_pose(pose):
    rospy.wait_for_service("set_pose")
    try:
        set_pose = rospy.ServiceProxy("set_pose", SetPose)
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        resp = set_pose(msg)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def publish_bypass(pose, ang_vel):
    pub_pose.publish(pose)
    pub_x.publish(pose.position.x)
    pub_y.publish(pose.position.y)
    pub_z.publish(pose.position.z)
    pub_ang_vel.publish(ang_vel)

    euler_dvlref_dvl = transformations.euler_from_quaternion(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )
    roll = euler_dvlref_dvl[0] * DEG_PER_RAD
    pitch = euler_dvlref_dvl[1] * DEG_PER_RAD
    yaw = euler_dvlref_dvl[2] * DEG_PER_RAD

    pub_theta_x.publish(roll)
    pub_theta_y.publish(pitch)
    pub_theta_z.publish(yaw)

    global last_time
    if rospy.Time.now() == last_time:
        return
    last_time = rospy.Time.now()

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "auv_base"
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation
    tf_broadcaster.sendTransform(t)

    t_rot = TransformStamped()
    t_rot.header.stamp = rospy.Time.now()
    t_rot.header.frame_id = "world_rotation"
    t_rot.child_frame_id = "auv_rotation"
    t_rot.transform.translation.x = 0
    t_rot.transform.translation.y = 0
    t_rot.transform.translation.z = 0
    t_rot.transform.rotation = pose.orientation
    tf_broadcaster.sendTransform(t_rot)

def cb_unity_state(msg):
    global reseted
    pose_x = msg.position.z
    pose_y = -msg.position.x
    pose_z = msg.position.y
    q_ESD_imunominaldown_x = msg.orientation.x
    q_ESD_imunominaldown_y = msg.orientation.y
    q_ESD_imunominaldown_z = msg.orientation.z
    q_ESD_imunominaldown_w = msg.orientation.w

    q_ESD_imunominaldown = np.quaternion(
        q_ESD_imunominaldown_w, q_ESD_imunominaldown_x, q_ESD_imunominaldown_y, q_ESD_imunominaldown_z
    )

    q_ENU_imunominalup = q_ENU_ESD * q_ESD_imunominaldown * q_imunominaldown_imunominalup

    twist_angular_e = msg.angular_velocity.z
    twist_angular_n = msg.angular_velocity.x
    twist_angular_u = -msg.angular_velocity.y
    twist_enu = [-twist_angular_e,twist_angular_n,twist_angular_u]


    frequencies = msg.frequencies
    times = [msg.times_pinger_1, msg.times_pinger_2, msg.times_pinger_3, msg.times_pinger_4]

    isDVLActive = msg.isDVLActive
    isIMUActive = msg.isIMUActive
    isDepthSensorActive = msg.isDepthSensorActive
    isHydrophonesActive = msg.isHydrophonesActive


    velocity_enu = [msg.velocity.z, -msg.velocity.x, msg.velocity.y]

    acceleration_enu = [msg.linear_acceleration.z, -msg.linear_acceleration.x,msg.linear_acceleration.y]

    # HYDROPHONES
    if isHydrophonesActive:
        for i in range(NUMBER_OF_PINGERS):
            hydrophones_msg = PingerTimeDifference()
            hydrophones_msg.frequency = frequencies[i]
            hydrophones_msg.times = times[i]
            pub_hydrophones_sensor.publish(hydrophones_msg)


    if bypass:
        pose = Pose()
        pose.position.x = pose_x
        pose.position.y = pose_y
        pose.position.z = pose_z

        quat = Quaternion(x=q_ENU_imunominalup.x, y=q_ENU_imunominalup.y, z=q_ENU_imunominalup.z, w=q_ENU_imunominalup.w)
        pose.orientation = quat

        ang_vel_auv = quaternion.rotate_vectors(
            q_ENU_imunominalup.inverse(), twist_enu
        )
        
        ang_vel = Vector3(*ang_vel_auv)

        publish_bypass(pose, ang_vel)
        
        return
    elif not reseted:
        pose = Pose()
        pose.position.x = pose_x
        pose.position.y = pose_y
        pose.position.z = pose_z

        quat = Quaternion(x=q_ENU_imunominalup.x, y=q_ENU_imunominalup.y, z=q_ENU_imunominalup.z, w=q_ENU_imunominalup.w)
        pose.orientation = quat
        reset_pose(pose)
        reseted = True

    # DVL - NWU
    if isDVLActive:

        q_ENU_dvlup =  q_ENU_imunominalup * q_imunominalup_dvlnominalup * q_dvlnominalup_dvlup

        velocity_dvl = quaternion.rotate_vectors(q_ENU_dvlup.inverse(), velocity_enu)

        dvl_msg = TwistWithCovarianceStamped()
        dvl_msg.twist.twist.linear = Vector3(*velocity_dvl)

        dvl_msg.header.stamp = rospy.Time.now()
        dvl_msg.header.frame_id = "dvl"

        pub_dvl_sensor.publish(dvl_msg)


    # IMU - NED
    if isIMUActive:
        imu_msg = Imu()

        q_ENU_imuup = q_ENU_imunominalup * q_imunominalup_imuup

        imu_msg.orientation = Quaternion(
            x=q_ENU_imuup.x, y=q_ENU_imuup.y, z=q_ENU_imuup.z, w=q_ENU_imuup.w
        )


        twist_imu = quaternion.rotate_vectors(
            q_ENU_imuup.inverse(), twist_enu
        )

        acceleration_imu = quaternion.rotate_vectors(
            q_ENU_imuup.inverse(), acceleration_enu
        )

        imu_msg.angular_velocity = Vector3(*twist_imu)
        imu_msg.linear_acceleration = Vector3(*acceleration_imu)
        
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu"

        pub_imu_sensor.publish(imu_msg)

    # DEPTH SENSOR
    if isDepthSensorActive:
        depth_msg = Float64() 
        depth_msg.data = pose_z

        pub_depth_sensor.publish(depth_msg)


if __name__ == "__main__":
    rospy.init_node("unity_bridge")

    bypass = not rospy.get_param("~ekf")
    reseted = False

    last_time = rospy.Time.now()

    tf_broadcaster = TransformBroadcaster()

    # Load parameters
    q_dvlnominalup_dvlup_w = rospy.get_param("q_dvlnominalup_dvlup_w")
    q_dvlnominalup_dvlup_x = rospy.get_param("q_dvlnominalup_dvlup_x")
    q_dvlnominalup_dvlup_y = rospy.get_param("q_dvlnominalup_dvlup_y")
    q_dvlnominalup_dvlup_z = rospy.get_param("q_dvlnominalup_dvlup_z")
    q_dvlnominalup_dvlup = np.quaternion(
        q_dvlnominalup_dvlup_w, q_dvlnominalup_dvlup_x, q_dvlnominalup_dvlup_y, q_dvlnominalup_dvlup_z
    )

    auv_dvl_offset_x = rospy.get_param("auv_dvl_offset_x")
    auv_dvl_offset_y = rospy.get_param("auv_dvl_offset_y")
    auv_dvl_offset_z = rospy.get_param("auv_dvl_offset_z")

    q_imunominalup_imuup_w = rospy.get_param("q_imunominalup_imuup_w")
    q_imunominalup_imuup_x = rospy.get_param("q_imunominalup_imuup_x")
    q_imunominalup_imuup_y = rospy.get_param("q_imunominalup_imuup_y")
    q_imunominalup_imuup_z = rospy.get_param("q_imunominalup_imuup_z")


    q_imunominalup_imuup = np.quaternion(
        q_imunominalup_imuup_w, q_imunominalup_imuup_x, q_imunominalup_imuup_y, q_imunominalup_imuup_z
    )

    q_imunominaldown_imunominalup = np.quaternion(0,1,0,0)
    q_imunominalup_dvlnominalup = np.quaternion(1,0,0,0)

    # REFERENCE FRAME DEFINITIONS
    q_ENU_ESD = np.quaternion(0, 1, 0, 0)


    q_imunominaldown_dvlnominalup = np.quaternion(0,1,0,0)


    pub_dvl_sensor = rospy.Publisher(
        "/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=1
    )
    pub_depth_sensor = rospy.Publisher("/sensors/depth/z", Float64, queue_size=1)
    pub_imu_sensor = rospy.Publisher(
        "/sensors/imu/data", Imu, queue_size=1
    )
    pub_hydrophones_sensor = rospy.Publisher(
        "/sensors/hydrophones/pinger_time_difference", 
        PingerTimeDifference, 
        queue_size=1
    )

    pub_pose = rospy.Publisher("/state/pose", Pose, queue_size=1)

    pub_x = rospy.Publisher("/state/x", Float64, queue_size=1)
    pub_y = rospy.Publisher("/state/y", Float64, queue_size=1)
    pub_z = rospy.Publisher("/state/z", Float64, queue_size=1)
    pub_theta_x = rospy.Publisher("/state/theta/x", Float64, queue_size=1)
    pub_theta_y = rospy.Publisher("/state/theta/y", Float64, queue_size=1)
    pub_theta_z = rospy.Publisher("/state/theta/z", Float64, queue_size=1)
    pub_ang_vel = rospy.Publisher("/state/angular_velocity", Vector3, queue_size=1)

    # Set up subscribers and publishers
    rospy.Subscriber("/unity/state", UnityState, cb_unity_state)

    rospy.spin()
