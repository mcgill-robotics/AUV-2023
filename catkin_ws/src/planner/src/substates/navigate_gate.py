import rospy
import smach
from .utility.functions import *
import threading
from std_msgs.msg import String

class NavigateGate(smach.State):
    def __init__(self, control, mapping, state, goThrough):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state
        self.goThrough = goThrough
        
        self.target_color = rospy.get_param("target_color")
        self.gate_width = rospy.get_param("gate_width")
        self.red_gate_side = rospy.get_param("red_side")

        if self.target_color not in ["red", "blue"]:
            raise ValueError("Target must be red or blue.")
        
        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("navigate_gate_time_limit")
        
        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("Gate Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, ud):
        print("Starting gate navigation.") 
        self.pub_mission_display.publish("Gate")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientationt.
        self.control.move((None, None, -1))
        self.control.flatten()

        gate_object = self.mapping.getClosestObject(cls="Gate", pos=(self.state.x, self.state.y))
        if gate_object is None:
            print("No gate in object map! Failed.")
            return "failure"
    
        print("Centering and rotating in front of gate.")
        offset_distance = -3
        gate_rot = 180 if gate_object[4] is None else gate_object[4]
        dtv = degreesToVector(gate_rot)
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 

        if self.timeout_occurred:
            return "timeout"
        self.control.rotateEuler((None,None,gate_rot)) # Bring to exact angle. 
        if self.timeout_occurred:
            return "timeout"
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # Move in front of gate.

        # Wait and repeat just to be safe.
        print("Waiting to improve measurement accuracy")
        rospy.sleep(rospy.get_param("object_observation_time"))

        print("Re-centering and rotating in front of gate.")
        self.mapping.updateObject(gate_object)
        gate_rot = 180 if gate_object[4] is None else gate_object[4]
        dtv = degreesToVector(gate_rot)
        offset = [] 
        for i in range(len(dtv)):
            offset.append(offset_distance * dtv[i]) 

        if self.timeout_occurred:
            return "timeout"
        self.control.rotateEuler((None,None,gate_rot)) # Bring to exact angle. 
        if self.timeout_occurred:
            return "timeout"
        self.control.move((gate_object[1] + offset[0], gate_object[2] + offset[1], gate_object[3])) # Move in front of gate.

        print("Successfully centered in front of gate")

        if not self.goThrough:
            return "success"

        print("Red is on the {} side. Target color is {}".format(self.red_gate_side, self.target_color))

        self.mapping.updateObject(gate_object)

        if self.timeout_occurred:
            return "timeout"
        if self.target_color == "red": 
            if self.red_gate_side == "left":
                print("Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # A quarter of gate width.
            else: 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # A quarter of gate width.
        else: 
            if self.red_gate_side == "right":
                print("Going through left side")
                self.control.moveDeltaLocal((0,self.gate_width/4,0)) # A quarter of gate width.
            else: 
                print("Going through right side")
                self.control.moveDeltaLocal((0,-self.gate_width/4,0)) # A quarter of gate width.

        if self.timeout_occurred:
            return "timeout"
        self.control.moveDeltaLocal((5.0,0.0,0.0))

        self.control.freeze_pose()
        self.thread_timer.cancel()
        print("Successfully passed through gate!")
        return "success"