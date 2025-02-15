#!/usr/bin/env python3
import smach
import rospy
import threading
from std_msgs.msg import String


class NavigateOctagon(smach.State):
    def __init__(self, control, mapping, state):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state

        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("octagon_time_limit")
        self.closeness_threshold = rospy.get_param("octagon_closeness_threshold")
        self.centering_dist_threshold = rospy.get_param("center_dist_threshold")
        self.centering_delta_increment = rospy.get_param("centering_delta_increment")

        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )
    
    def timer_thread_func(self):
        self.pub_mission_display.publish("Octagon Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def is_close(self, current_pos, target_pos, threshold): #better place to put this function???
        distance = ((current_pos[0] - target_pos[0]) ** 2 + (current_pos[1] - target_pos[1]) ** 2) ** 0.5
        return distance < threshold
    
    def execute(self, ud):
        print("Starting octagon navigation.")
        self.pub_mission_display.publish("Octagon")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        self.control.flatten()

        started = False
        while not self.timeout_occurred:
            octagon_obj = self.mapping.getClosestObject((self.state.x, self.state.y), cls="Octagon Table")
            
            if octagon_obj is None: #TODO: better way to handle this?
                print("No octagon in object map! Failed.")
                return "failure"
            
            print("Moving towards the center of the octagon.") if not started else None
            started = True

            self.control.move(
                (octagon_obj[1], octagon_obj[2], rospy.get_param("down_cam_search_depth")),
                face_destination=True,
            )

            #If close enough to the octagon that we estimate vision can accurately localize it, break
            if self.is_close((self.state.x, self.state.y), (octagon_obj[1], octagon_obj[2]), self.closeness_threshold):
                rospy.sleep(5)
                self.control.move(
                (octagon_obj[1], octagon_obj[2], rospy.get_param("down_cam_search_depth")),
                face_destination=True,
                )
                # center distance loop
                while (self.mapping.distance > self.centering_dist_threshold):
                    if self.timeout_occurred:
                        return "timeout"
                    if self.mapping.delta_height < 0:
                        self.control.moveDeltaLocal((self.centering_delta_increment, 0, 0))
                    elif self.mapping.delta_height > 0:
                        self.control.moveDeltaLocal((-self.centering_delta_increment, 0, 0))
                    if self.mapping.delta_width < 0:
                        self.control.moveDeltaLocal((0, self.centering_delta_increment, 0))
                    elif self.mapping.delta_width > 0:
                        self.control.moveDeltaLocal((0, -self.centering_delta_increment, 0))
                    rospy.sleep(3)
                print("Centered")
                print("Surfacing.")
                self.control.flatten()
                self.control.kill()
                break
        else:
            return "timeout"

        print("Successfully navigated the octagon.")
        return "success"
    


