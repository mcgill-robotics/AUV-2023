#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
from substates.utility.functions import countdown

def main():
    rospy.init_node("pooltest")

    controls = Controller(rospy.Time(0))

    try:
        # Reset orientation to default
        rospy.loginfo("Resetting orientation to default...")
        controls.rotateEuler([0, 0, 0])
        rospy.sleep(1)  # Give time for the rotation to complete

        # Move down by 0.5 meters
        rospy.loginfo("Moving down by 0.5 meters...")
        controls.moveDelta([0, 0, -0.5])
        rospy.sleep(2)  # Adjust sleep duration to ensure the motion completes

        # Move in a square pattern locally
        rospy.loginfo("Executing square pattern movement...")
        controls.moveDeltaLocal([0.5, 0, 0])
        rospy.sleep(2)

        controls.moveDeltaLocal([0, 0.5, 0])
        rospy.sleep(2)

        controls.moveDeltaLocal([-0.5, 0, 0])
        rospy.sleep(2)

        controls.moveDeltaLocal([0, -0.5, 0])
        rospy.sleep(2)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls...")
        controls.kill()  # Ensure the robot is stopped

if __name__ == "__main__":
    main()
