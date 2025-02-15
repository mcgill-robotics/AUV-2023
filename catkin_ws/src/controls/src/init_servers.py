#!/usr/bin/env python3

import rospy

from servers.effort_server import EffortServer
from servers.state_quaternion_pid_server import StateQuaternionServer

# define preempt callbacks using the cancel methods. This is necessary because action lib does not
# allow methods to be callback function for preempting.
if __name__ == "__main__":
    rospy.init_node("server manager")
    sup = EffortServer()
    sup.server.register_preempt_callback(lambda: sup.cancel())
    quat = StateQuaternionServer()
    quat.server.register_preempt_callback(lambda: quat.cancel())
    rospy.spin()
