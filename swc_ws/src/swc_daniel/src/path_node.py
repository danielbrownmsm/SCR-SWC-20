#!/usr/bin/env python

import rospy
import PathHandler
from swc_msgs.msg import Control, State
from swc_msgs.srv import Waypoints

_control_pub = None

def main():
    global _control_pub
    global pathHandler

    # Initalize our node in ROS
    rospy.init_node("path_node")

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service("/sim/waypoints")
    waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()

    pathHandler = PathHandler.PathHandler(waypoints.waypoints)

    # Create a timer that calls timer_callback() with a period of 0.1
    rospy.Timer(rospy.Duration(0.1), publish)
    
    # get sensor data
    rospy.Subscriber("/daniel/state", State, pathHandler.stateCallback)
    #rospy.Subscriber("/daiel/path", Control, pathHandler.pathCallback)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

def publish(event):
    # Publish the message to /daniel/state so the simulator receives it
    global _control_pub # globals because all funcs and stuff
    global pathHandler

    _control_pub.publish(pathHandler.getMessage())

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass