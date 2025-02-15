#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import String
import ast

from missions_utils import *
from substates.breadth_first_search import *
from substates.in_place_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *
from substates.trick import *
from substates.navigate_gate import *
from substates.navigate_buoy import *
from substates.navigate_pinger import *
from substates.navigate_bin import *
from substates.octagon_task import *
from substates.quali import *


class Missions:
    def __init__(self):
        self.mapping = ObjectMapper()
        self.state = StateTracker()
        self.control = Controller(rospy.Time(0))

        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def gate(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_gate" + count
        third_state_name = "tricks_gate" + count
        fourth_state_name = "navigate_gate_go_through" + count
        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )
        if mission_after_timeout is not None:
            target_timeout_state_name = mission_after_timeout
        else:
            target_timeout_state_name = (
                "failure"
                if target_success_state_name == "success"
                else target_success_state_name
            )

        smach.StateMachine.add(
            first_state_name,
            InPlaceSearch(
                self.control, self.mapping, target_class="Gate", min_objects=1
            ),
            transitions={
                "success": second_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateGate(
                control=self.control,
                mapping=self.mapping,
                state=self.state,
                goThrough=False,
            ),
            transitions={
                "success": third_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )
        smach.StateMachine.add(
            third_state_name,
            Trick(self.control),
            transitions={
                "success": fourth_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )
        smach.StateMachine.add(
            fourth_state_name,
            NavigateGate(
                control=self.control,
                mapping=self.mapping,
                state=self.state,
                goThrough=True,
            ),
            transitions={
                "success": target_success_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

    def lane_marker(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_lane_marker" + count
        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )
        if mission_after_timeout is not None:
            target_timeout_state_name = mission_after_timeout
        else:
            target_timeout_state_name = (
                "failure"
                if target_success_state_name == "success"
                else target_success_state_name
            )

        smach.StateMachine.add(
            first_state_name,
            BreadthFirstSearch(
                self.control,
                self.mapping,
                target_class="Lane Marker",
                min_objects=int(count),
            ),
            transitions={
                "success": second_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateLaneMarker(self.control, self.mapping, self.state, origin_class=""),
            transitions={
                "success": target_success_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

    def pinger(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm
        global missions_to_objects
        global missions_to_pinger_frequency
        
        advance_distance = int(rospy.get_param("advance_distance"))
        update_heading_time = int(rospy.get_param("update_heading_time"))
        update_heading_time = rospy.Duration(update_heading_time)
        
        first_state_name = first_state_name + count
        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )
        if mission_after_timeout is not None:
            target_timeout_state_name = mission_after_timeout
        else:
            target_timeout_state_name = (
                "failure"
                if target_success_state_name == "success"
                else target_success_state_name
            )

        smach.StateMachine.add(
            first_state_name,
            NavigatePinger(
                control=self.control,
                state=self.state,
                mapping=self.mapping,
                goal_object=missions_to_objects[mission_after_success[:-1]],
                advance_distance=advance_distance,
                pinger_frequency=missions_to_pinger_frequency[mission_after_success[:-1]],
                update_heading_time=update_heading_time,
            ),
            transitions={
                "success": target_success_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

    def buoy(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_buoy" + count
        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )
        if mission_after_timeout is not None:
            target_timeout_state_name = mission_after_timeout
        else:
            target_timeout_state_name = (
                "failure"
                if target_success_state_name == "success"
                else target_success_state_name
            )

        smach.StateMachine.add(
            first_state_name,
            InPlaceSearch(
                self.control, self.mapping, target_class="Buoy", min_objects=1
            ),
            transitions={
                "success": second_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateBuoy(self.control, self.mapping, self.state),
            transitions={
                "success": target_success_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

    def octagon(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_octagon" + count

        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )
        if mission_after_timeout is not None:
            target_timeout_state_name = mission_after_timeout
        else:
            target_timeout_state_name = (
                "failure"
                if target_success_state_name == "success"
                else target_success_state_name
            )

        smach.StateMachine.add(
            first_state_name,
            InPlaceSearch(
                self.control, self.mapping, target_class="Octagon Table", min_objects=1
            ),
            transitions={
                "success": second_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

        smach.StateMachine.add(
            second_state_name,
            NavigateOctagon(self.control, self.mapping, self.state),
            transitions={
                "success": target_success_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

    def torpedo(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm

        """
        ADD SOME TRANSITIONS HERE
        """

    def trick(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm

        first_state_name = first_state_name + count
        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )
        if mission_after_timeout is not None:
            target_timeout_state_name = mission_after_timeout
        else:
            target_timeout_state_name = (
                "failure"
                if target_success_state_name == "success"
                else target_success_state_name
            )

        smach.StateMachine.add(
            first_state_name,
            Trick(self.control),
            transitions={
                "success": target_success_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

    def bins(
        self, first_state_name, count, mission_after_success, mission_after_timeout
    ):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_bin" + count
        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )
        if mission_after_timeout is not None:
            target_timeout_state_name = mission_after_timeout
        else:
            target_timeout_state_name = (
                "failure"
                if target_success_state_name == "success"
                else target_success_state_name
            )
            
        smach.StateMachine.add(
            first_state_name,
            LinearSearch(
                self.control, self.mapping, target_class="Bin", min_objects=1
            ),
            transitions={
                "success": second_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )
        
        smach.StateMachine.add(
            second_state_name,
            NavigateDropper(self.control, self.mapping, self.state),
            transitions={
                "success": target_success_state_name,
                "timeout": target_timeout_state_name,
                "failure": "failure",
            },
        )

    def quali(self, first_state_name, count, mission_after_success, _):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_quali" + count
        target_success_state_name = (
            mission_after_success if mission_after_success is not None else "success"
        )

        smach.StateMachine.add(
            first_state_name,
            NavigateGate(
                control=self.control,
                mapping=self.mapping,
                state=self.state,
                goThrough=False,
            ),
            transitions={
                "success": second_state_name,
                "timeout": second_state_name,
                "failure": "failure",
            },
        )
        smach.StateMachine.add(
            second_state_name,
            Quali(control=self.control),
            transitions={
                "success": target_success_state_name,
                "timeout": "failure",
                "failure": "failure",
            },
        )

    def endPlanner(self, msg="Shutting down mission planner."):
        self.pub_mission_display.publish("End")
        print(msg)
        self.control.kill()


if __name__ == "__main__":
    rospy.init_node("competition_planner")
    
    # Retrieve dictionnaries
    missions_to_objects_str = rospy.get_param("missions_to_objects")
    missions_to_pinger_frequency_str = rospy.get_param("missions_to_pinger_frequency")
    missions_to_objects = ast.literal_eval(missions_to_objects_str)
    missions_to_pinger_frequency = ast.literal_eval(missions_to_pinger_frequency_str)

    sm = None
    missions = Missions()

    rospy.on_shutdown(missions.endPlanner)

    mission_options = [
        # (mission name, mission function, default first state name, num of times selected by user).
        ["Gate", missions.gate, "find_gate", 0],
        ["Lane Marker", missions.lane_marker, "find_lane_marker", 0],
        ["Trick", missions.trick, "trick", 0],
        ["Buoy", missions.buoy, "find_buoy", 0],
        ["Octagon", missions.octagon, "find_octagon", 0],
        ["Pinger", missions.pinger, "navigate_pinger", 0],
        ["Torpedo", missions.torpedo, "", 0],
        ["Bins", missions.bins, "find_bin", 0],
        ["Quali", missions.quali, "navigate_gate_not_through", 0],
        ["Competition", None, None, 0],
    ]

    try:
        done = False
        while not done:
            missions_selected = get_user_missions_selected(mission_options)

            sm = smach.StateMachine(outcomes=["success", "failure", "timeout"])
            sm.open()

            for i in range(len(missions_selected)):
                (
                    current_mission_name,
                    current_mission_count,
                    mission_after_success,
                    mission_after_timeout,
                ) = get_state_params(mission_options, missions_selected, i)

                # Add task to state machine.
                mission_options[missions_selected[i]][1](
                    first_state_name=current_mission_name,
                    count=str(current_mission_count),
                    mission_after_success=mission_after_success,
                    mission_after_timeout=mission_after_timeout,
                )

            # Execute state machine.
            res = sm.execute()
            sm.close()
            missions.endPlanner(
                "Finished planner with result: " + str(res) + "!!!!!!!!!"
            )

    except KeyboardInterrupt:
        missions.endPlanner()

    rospy.spin()
