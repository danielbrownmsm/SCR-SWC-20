#!/usr/bin/env python

import rospy
import ControlHandler
from swc_msgs.msg import Control
from swc_msgs.msg import State

_control_pub = None

def main():
    global _control_pub
    global controlHandler

    # Initalize our node in ROS
    rospy.init_node("control_node")

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _control_pub = rospy.Publisher("/sim/control", Control, queue_size=3)

    controlHandler = ControlHandler.ControlHandler()

    # Create a timer that calls timer_callback() with a period of 0.1
    rospy.Timer(rospy.Duration(0.1), publish)
    
    # get sensor data
    #rospy.Subscriber("/daniel/state", State, controlHandler.stateCallback)
    rospy.Subscriber("/daiel/path", Control, controlHandler.pathCallback)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

def publish(event):
    # Publish the message to /daniel/state so the simulator receives it
    global _control_pub # globals because all funcs and stuff
    global controlHandler

    _control_pub.publish(controlHandler.getMessage())

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass