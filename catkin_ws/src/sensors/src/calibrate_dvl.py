#!/usr/bin/env python3

import rospy
import serial


def main():
    rospy.init_node("calibrate_dvl")

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")

    conn = serial.Serial(port)
    conn.timeout = 10
    # dvl's baud has been set to 115200 but its default is 9600.
    # There is a way to set the baudrate of the dvl through a command.
    conn.baudrate = baudrate

    if not conn.isOpen():
        conn.open()

    conn.send_break()
    conn.flush()

    print("Calibrating DVL...")

    conn.write("wcg\r\n".encode("utf-8"))
    conn.flush()

    while conn.is_open and not rospy.is_shutdown():
        try:
            line = conn.readline().decode("utf-8")
            if line.startswith("wra"):
                print("INFO: DVL gyro calibration successful.")
                break
            elif line.startswith("wrn"):
                print("WARN: DVL gyro calibration failed.")
                break
        except Exception as e:
            print(e)
            break


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
