#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion

from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgEkfQuat, SbgImuData
from std_msgs.msg import Float64
from tf import transformations

Q_NWU_NED = np.quaternion(0, 1, 0, 0)
DEG_PER_RAD = 180 / np.pi
RAD_PER_DEG = 1 / DEG_PER_RAD

class Sensor():
    def __init__(self, sensor_name):
        self.time_before_considered_inactive = 1 #seconds
        self.sensor_name = sensor_name

        self.x = 0
        self.y = 0
        self.z = 0
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.quaternion = np.quaternion(1, 0, 0, 0)
        self.angular_velocity = np.array([0,0,0])

        # initialize a sensor as "inactive"
        self.last_unique_state_time = 0
        self.last_state = [self.x,self.y,self.z,self.roll,self.pitch,self.yaw,self.quaternion,self.angular_velocity]
    
    def updateLastState(self):
        current_state = [self.x,self.y,self.z,self.roll,self.pitch,self.yaw,self.quaternion,self.angular_velocity]
        if current_state != self.last_state:
            if rospy.get_time() - self.last_unique_state_time > self.time_before_considered_inactive:
                rospy.loginfo("{} has become active.".format(sensor_name))
            self.last_unique_state_time = rospy.get_time() 
        self.last_state = current_state
        
    def isActive(self):
        if rospy.get_time() - self.last_unique_state_time > self.time_before_considered_inactive:
            rospy.logwarn("{} has been inactive for {} seconds.".format(sensor_name, time_before_considered_inactive))
            return False
        else:
            return True        

class DepthSensor(Sensor):
    def __init__(self):
        super().__init__("Depth Sensor")
        self.z_pos_mount_offset = 0
        rospy.Subscriber("/depth", Float64, self.depth_cb)

    def depth_cb(self, depth_msg):
        self.z = depth_msg.data + self.z_pos_mount_offset
        self.updateLastState()

class IMU(Sensor):
    def __init__(self):
        super().__init__("IMU")
        self.imu_auv = np.quaternion(0, 0, 0, 1) # imu is rotated 180 degrees about z axis relative to AUV frame
        rospy.Subscriber("/sbg/imu_data", SbgImuData, self.ang_vel_cb)
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.quat_cb)
        
    def ang_vel_cb(self, msg):
        # angular velocity vector relative to imu frame 
        ang_vel_imu = np.array([msg.gyro.x, -msg.gyro.y, -msg.gyro.z])
        # anuglar velocity vector relative to AUV frame 
        self.angular_velocity = quaternion.rotate_vectors(self.quat_mount_offset, ang_vel_imu)
        self.updateLastState()

    def quat_cb(self, msg):
        q_ned_imu = np.quaternion(msg.quaternion.w, msg.quaternion.x, msg.quaternion.y, msg.quaternion.z)
        q_nwu_imu = Q_NWU_NED * q_ned_imu
        self.nwu_auv = q_nwu_imu * self.imu_auv

        # we should move away from this
        np_quaternion = np.array([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
        self.roll = transformations.euler_from_quaternion(np_quaternion, 'rxyz')[0] * DEG_PER_RAD
        self.pitch = transformations.euler_from_quaternion(np_quaternion, 'ryxz')[0] * DEG_PER_RAD
        self.yaw = transformations.euler_from_quaternion(np_quaternion, 'rzyx')[0] * DEG_PER_RAD
        self.updateLastState()

class DVL(Sensor):
    def __init__(self, imu):
        super().__init__("DVL")
        self.quat_mount_offset = np.quaternion(0, 0.3826834, 0.9238795, 0) # RPY [deg]: (180, 0, -135) 
        self.pos_mount_offset = np.array([0.0, 0.0, -0.3])

        self.imu = imu
        self.dvl_ref_frame = None
        rospy.Subscriber("/dead_reckon_report", DeadReckonReport, self.dr_cb)
    
    def dr_cb(self, dr_msg):
        # quaternion/position of dvl relative to dvlref 
        q_dvlref_dvl = transformations.quaternion_from_euler(dr_msg.roll*RAD_PER_DEG, dr_msg.pitch*RAD_PER_DEG, dr_msg.yaw*RAD_PER_DEG)
        q_dvlref_dvl = np.quaternion(q_dvlref_dvl[3], q_dvlref_dvl[0], q_dvlref_dvl[1], q_dvlref_dvl[2]) # transformations returns quaternion as nparray [x, y, z, w]
        q_dvlref_auv = q_dvlref_dvl * self.q_dvl_auv
        pos_dvlref = np.array([dr_msg.x, dr_msg.y, dr_msg.z])

        #update dvl ref frame using imu
        if self.imu.isActive():
            q_nwu_auv = self.imu.nwu_auv
            q_dvlref_nwu = q_dvlref_auv * q_nwu_auv.inverse() 

        self.pos_auv = quaternion.rotate_vectors(q_dvlref_nwu, pos_dvlref) + self.pos_mount_offset

        # if self.dvl_ref_frame is None: return

        # # quaternion of AUV from DVL
        # self.quaternion = (self.dvl_ref_frame * q_dvl_dvlref) * self.quat_mount_offset.inverse()
        # # get roll, pitch, yaw
        # np_quaternion = np.array([self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
        # roll = transformations.euler_from_quaternion(np_quaternion, 'rxyz')[0] * DEG_PER_RAD
        # pitch = transformations.euler_from_quaternion(np_quaternion, 'ryxz')[0] * DEG_PER_RAD
        # yaw = transformations.euler_from_quaternion(np_quaternion, 'rzyx')[0] * DEG_PER_RAD
        # # calculate angular velocity (not super precise but will only be used when IMU is inactive)
        # dt = (rospy.get_time() - self.last_unique_state_time)
        # ang_vel_x = (roll - self.roll) * RAD_PER_DEG / dt
        # ang_vel_y = (pitch - self.pitch) * RAD_PER_DEG / dt
        # ang_vel_z = (yaw - self.yaw) * RAD_PER_DEG / dt
        # self.angular_velocity = np.array([ang_vel_x, ang_vel_y, ang_vel_z])
        # self.roll = roll
        # self.pitch = pitch
        # self.yaw = yaw

        # # position of AUV from DVL
        # mount_offset = quaternion.rotate_vectors(self.quaternion, self.pos_mount_offset)
        # position = quaternion.rotate_vectors(self.dvl_ref_frame.inverse(), pos_dvl_dvlref) + mount_offset
        # self.x, self.y, self.z = position

        # self.updateLastState()
