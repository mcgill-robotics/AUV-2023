import rospy
import numpy as np
from sensor_msgs.msg import Imu

def imu_cb(msg):
    global ang_vel, accel
    ang_vel.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    accel.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

def finish(event):
    global ang_vel, accel
    imu_sub.unregister()
    ang_vel = np.array(ang_vel)
    accel = np.array(accel)
    ang_vel_cov = np.cov(ang_vel.T)
    accel_cov = np.cov(accel.T)
    print('Angular Velocity Covariance:')
    print(ang_vel_cov)
    print('Linear Acceleration Covariance:')
    print(accel_cov)

    rospy.signal_shutdown('Finished')

if __name__ == '__main__':
    rospy.init_node('sensor_covariance')
    ang_vel = []
    accel = []
    imu_sub = rospy.Subscriber('/sensors/imu/data', Imu, imu_cb)
    timer = rospy.Timer(rospy.Duration(30), finish)
    rospy.spin()


