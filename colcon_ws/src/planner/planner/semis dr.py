#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy import Duration
from substates.utility.functions import countdown
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.functions import *

def main(args=None):
    START2OCT_X = 1.4 + 2.74 * 6.5
    START2OCT_Y = 1.5 * 2.74
    VELOCITY = 1

    rclpy.init(args=args)
    node = Node("semis")

    pub_mission_display = node.create_publisher(String, "/mission_display", 1)


    def update_display(status):
        pub_mission_display.publish(status)
        node.get_logger().info(status)


    countdown(120)

    update_display("INIT")
    state = StateTracker()
    controls = Controller(Time(seconds=0))


    def moveForward(distance):
        time_to_sleep = distance / VELOCITY
        controls.forceLocal((20, 0))
        node.get_clock().sleep_for(Duration(seconds=time_to_sleep))
        controls.forceLocal((0, 0))


    update_display("MOVE2GATE")
    controls.rotateDeltaEuler([0, 0, 0])  # TODO: change rotation based on flip coin
    angle_to_gate = state.theta_z
    controls.moveDelta([None, None, -0.75])
    moveForward(1)


    update_display("TRICKS")
    for _ in range(2 * 3):
        controls.rotateDeltaEuler((0, 0, 120))


    update_display("GATE + NAV TO OCT")
    moveForward(START2OCT_X - 1)
    controls.rotateDeltaEuler((0, 0, -90))
    moveForward(START2OCT_Y)

    controls.kill()

if __name__ == "__name__":
    main()
