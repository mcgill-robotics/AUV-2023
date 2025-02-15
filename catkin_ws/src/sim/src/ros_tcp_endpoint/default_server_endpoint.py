#!/usr/bin/env python

import rospy

from server import TcpServer


def main(args=None):
    # Start the Server Endpoint
    rospy.init_node("unity_endpoint", anonymous=True)
    tcp_server = TcpServer(rospy.get_name())
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    rospy.sleep(1)
    main()
