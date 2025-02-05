#!/usr/bin/env python3
<<<<<<< HEAD
=======
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy import Duration
from substates.utility.controller import Controller
from auv_msgs.msg import ThrusterMicroseconds
from std_msgs.msg import Float64
>>>>>>> 6b75633b3de439bad49d7369c337618fe9917ea9
import keyboard
import pickle
import time

from substates.utility.controller import Controller
from auv_msgs.msg import ThrusterMicroseconds
from std_msgs.msg import Float64

import rclpy
from rclpy.node import Node

# forces produced by T200 thruster at 14V (N)
MAX_FWD_FORCE = 4.52 * 9.81
MAX_BKWD_FORCE = -3.52 * 9.81

class JoyStick(Node):
    def __init__(self):
        super().__init__('joystick')
        self.reset_cmd = ThrusterMicroseconds([1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
        self.get_clock().sleep_for(Duration(seconds=7))

        self.x_pub = self.create_publisher("/controls/force/surge", Float64, queue_size=1)
        self.y_pub = self.create_publisher("/controls/force/sway", Float64, queue_size=1)
        self.z_pub = self.create_publisher("/controls/force/global/z", Float64, queue_size=1)
        self.roll_pub = self.create_publisher("/controls/torque/roll", Float64, queue_size=1)
        self.pitch_pub = self.create_publisher("/controls/torque/pitch", Float64, queue_size=1)
        self.yaw_pub = self.create_publisher("/controls/torque/yaw", Float64, queue_size=1)
        self.pub = self.create_publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)
        self.controls = Controller(rclpy.time.Time())
        self.RECORDING = []

    def reset_thrusters(self):
        self.pub.publish(self.reset_cmd)
        self.get_logger().info("Safely shutting down thrusters")

    def record_keyboard_state(self):
        keyboard_state = []
        if keyboard.is_pressed("space"):
            keyboard_state.append("space")
        if keyboard.is_pressed("w"):
            keyboard_state.append("w")
        if keyboard.is_pressed("s"):
            keyboard_state.append("s")
        if keyboard.is_pressed("a"):
            keyboard_state.append("a")
        if keyboard.is_pressed("d"):
            keyboard_state.append("d")
        if keyboard.is_pressed("q"):
            keyboard_state.append("q")
        if keyboard.is_pressed("e"):
            keyboard_state.append("e")
        if keyboard.is_pressed("o"):
            keyboard_state.append("o")
        if keyboard.is_pressed("u"):
            keyboard_state.append("u")
        if keyboard.is_pressed("i"):
            keyboard_state.append("i")
        if keyboard.is_pressed("k"):
            keyboard_state.append("k")
        if keyboard.is_pressed("j"):
            keyboard_state.append("j")
        if keyboard.is_pressed("l"):
            keyboard_state.append("l")
        if keyboard.is_pressed("1"):
            keyboard_state.append("1")
        if keyboard.is_pressed("2"):
            keyboard_state.append("2")
        if keyboard.is_pressed("r"):
            keyboard_state.append("r")
        if keyboard.is_pressed("t"):
            keyboard_state.append("t")
        
        self.RECORDING.append(keyboard_state)

    def joystick(self, keyboard_state=None):
<<<<<<< HEAD
        """
        Detects keyboard key presses and react by publishing the PWM force variables.

        Note: **Esc** kills controls, and should be used with caution.

        Refer to the wiki for more documentation.

        Arguments:
            keyboard_state: State of the keyboard

        Returns:    
            boolean
        """
=======
>>>>>>> 6b75633b3de439bad49d7369c337618fe9917ea9
        desired_x_force = 0
        desired_y_force = 0
        desired_z_force = 0
        desired_x_torque = 0
        desired_y_torque = 0
        desired_z_torque = 0        

        if keyboard_state is None:
            self.record_keyboard_state()
            keyboard_state = []

        space_pressed = keyboard.is_pressed("space") or "space" in keyboard_state
        current_force_amt = 0.5 if space_pressed else 0.1

        # Detect various key presses on the keyboard
        if keyboard.is_pressed("esc"):
            self.controls.kill()
            return False
        if keyboard.is_pressed("w") or "w" in keyboard_state:
            desired_x_force += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("s") or "s" in keyboard_state:
            desired_x_force += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("a") or "a" in keyboard_state:
            desired_y_force += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("d") or "d" in keyboard_state:
            desired_y_force += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("q") or "q" in keyboard_state:
            desired_z_force += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("e") or "e" in keyboard_state:
            desired_z_force += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("o") or "o" in keyboard_state:
            desired_y_torque += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("u") or "u" in keyboard_state:
            desired_y_torque += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("i") or "i" in keyboard_state:
            desired_y_torque += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("k") or "k" in keyboard_state:
            desired_y_torque += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("j") or "j" in keyboard_state:
            desired_z_torque += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("l") or "l" in keyboard_state:
            desired_z_torque += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("r") or "r" in keyboard_state:
            self.get_logger().info("STARTING ROTATION!!!")
            self.controls.rotateDeltaEuler([0, 0, 150])
            self.get_logger().info("DONE ROTATING!!!!")
        if keyboard.is_pressed("t") or "t" in keyboard_state:
            self.get_logger().info("STARTING ROTATION!!!")
            self.controls.rotateDeltaEuler([0, 0, -150])
            self.get_logger().info("DONE ROTATING!!!!")
        if keyboard.is_pressed("1") or "1" in keyboard_state:
            self.get_logger().info("STARTING ROTATION!!!")
            self.controls.rotateDeltaEuler([0, 0, 90])
            self.get_logger().info("DONE ROTATING!!!!")
        if keyboard.is_pressed("2") or "2" in keyboard_state:
            self.get_logger().info("STARTING ROTATION!!!")
            self.controls.rotateDeltaEuler([0, 0, -90])
            self.get_logger().info("DONE ROTATING!!!!")

        self.x_pub.publish(desired_x_force)
        self.y_pub.publish(desired_y_force)
        # self.z_pub.publish(desired_z_force)
        # self.roll_pub.publish(desired_x_torque)
        # self.pitch_pub.publish(desired_y_torque)
        # self.yaw_pub.publish(desired_z_torque)

        return True
    
    def run_recording(self):
        is_recording_res = input("Recording?")
        is_recording = is_recording_res.lower() == "y"
        self.get_logger().info("SUBMERGING...")
        self.controls.move([None, None, -0.5])
        self.get_logger().info("FLATTENING...")
        self.controls.rotateEuler([0,0,-137])

        self.get_logger().info("\n\n\n")

        self.get_logger().info(" > R to make a 90 deg. YAW")
        self.get_logger().info(" > T to make a -90 deg. YAW")
        self.get_logger().info(" > 1 to make a 30 deg. YAW")
        self.get_logger().info(" > 2 to make a -30 deg. YAW")
        self.get_logger().info(" > WASD for SURGE/SWAY")
        self.get_logger().info(" > Q/E for UP/DOWN")
        self.get_logger().info(" > IJKL for PITCH/YAW")
        self.get_logger().info(" > U/O for ROLL")
        self.get_logger().info(" > hold SPACE for max. force")
        self.get_logger().info(" > ESC to exit cleanly")

        if is_recording:
            while rclpy.ok():
                stay_alive = self.joystick()
                self.get_clock().sleep_for(Duration(seconds=0.01))
                if not stay_alive:
                    break
            with open("keyboard_rec.pkl", "wb") as f:
                self.node.get_logger().info("SAVING")
                pickle.dump(RECORDING, f) 
        else:
            with open("keyboard_rec.pkl", "rb") as f:
                RECORDING = pickle.load(f)
            for keyboard_state in RECORDING:
                self.joystick(keyboard_state)
                self.get_clock().sleep_for(Duration(seconds=0.01))

        self.controls.kill()

        # self.controls.freeze_pose()
        # self.controls.freeze_position()
        # self.controls.freeze_rotation()
        # self.controls.move([0, 0, -2])
        # self.controls.moveDeltaLocal([0, 0, -1])
        # self.controls.rotate([1, 0, 0, 0])
        # self.controls.rotateDelta([1, 0, 0, 0])
        # self.controls.rotateEuler([0, 0, 180])
        # self.controls.rotateDeltaEuler([0, 0, 180])


def main(args=None):
    rclpy.init(args=args)

    node = JoyStick()
    node.get_logger().info("NOTE: Launch controls.launch and propulsion.launch to use joystick.")

    try:
        node.run_recording()
    except KeyboardInterrupt:
        pass 
    finally:
        node.reset_thrusters()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__name__':
    main()