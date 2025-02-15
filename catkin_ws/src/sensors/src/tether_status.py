#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Int32


def is_tether_active(_):
     # Just for linux. If you want Windows, change
     # -c to -n
     command = f"ping -c 1 {ip_address} > /dev/null" 
     response = os.system(command)
     # "response == 0" = successful 
     if response == 0:
          pub_tether_status.publish(1)
     else:
          pub_tether_status.publish(0)
     

if __name__ == "__main__":
     rospy.init_node("tether_status")

     pub_tether_status = rospy.Publisher(
          "/tether/status", Int32, queue_size=1
     )

     ip_address = rospy.get_param("ip_address_tether")
     ping_interval = rospy.get_param("ping_interval")
     timer = rospy.Timer(rospy.Duration(ping_interval), is_tether_active)

     rospy.on_shutdown(timer.shutdown)

     rospy.spin()