#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from tf import transformations
import numpy as np

RAD_PER_DEG = np.pi / 180.0


def parse_dead_reckon_report(line, quat_variance):
    tokens = line.split(",")
    time_stamp = float(tokens[1])
    x = float(tokens[2])
    y = float(tokens[3])
    z = float(tokens[4])
    std = float(tokens[5])
    roll = float(tokens[6])
    pitch = float(tokens[7])
    yaw = float(tokens[8])
    status = bool(tokens[9])
    return [roll, pitch, yaw]



def main():
    rospy.init_node("waterlinked_driver")

    pub_vr = rospy.Publisher("/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=1)
    pub_dr = rospy.Publisher("/sensors/dvl/pose", PoseWithCovarianceStamped, queue_size=1)

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")
    quat_variance = rospy.get_param("~quat_variance")


    try:
        conn = serial.Serial(port)
    except serial.serialutil.SerialException:
        rospy.logerr("ERR: /dev/dvl directory does not exist")
        rospy.sleep(5)
        exit()

    conn.timeout = 10
    # dvl's baud has been set to 115200 but its default is 9600.
    # There is a way to set the baudrate of the dvl through a command.
    conn.baudrate = baudrate

    if not conn.isOpen():
        conn.open()

    conn.send_break()
    conn.flush()

    conn.write("wcr\r\n".encode("utf-8"))
    conn.flush()

    print("Reset dead reckoning.")

    while conn.is_open and not rospy.is_shutdown():
        try:
            line = conn.readline().decode("utf-8")
            if line.startswith("wra"):
                print("INFO: DVL dead reckoning reset successful.")
                break
            elif line.startswith("wrn"):
                print("WARN: DVL dead reckoning reset failed.")
                break
        except Exception as e:
            print(e)
            break

    # Only grabbing data we care about but this dvl can be used for more.
    # Refer to work horse manual for more info
    start = rospy.Time.now()
    eulers = []
    while conn.is_open and not rospy.is_shutdown() and rospy.Time.now() - start < rospy.Duration(30):
        try:
            line = conn.readline().decode("utf-8")
            # print(line)
            if line.startswith("wrp"):
                eulers.append(parse_dead_reckon_report(line, quat_variance))
        except Exception as e:
            print(e)
            conn.close()
            exit()
    eulers = np.array(eulers)
    eulers_cov = np.cov(eulers.T)
    print('Eulers Covariance:')
    print(eulers_cov)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
