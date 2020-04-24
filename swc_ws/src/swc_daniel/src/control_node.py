#!/usr/bin/env python

import rospy
from swc_msgs.msg import Control
from swc_msgs.msg import Position
from std_msgs.msg import Bool
import ControlHandler as CH

_control_pub = None

def timer_callback(event):
    # Publish the message to /sim/control
    _control_pub.publish(ch.getMessage())

def main():
    global _control_pub
    global ch

    # Initalize our node in ROS
    rospy.init_node('control_node.py')

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    ch = CH.ControlHandler()

    # Create a timer that calls timer_callback() with a period of 0.1
    rospy.Timer(rospy.Duration(0.1), timer_callback)

    # get processed data
    rospy.Subscriber("/sim/position", Position, ch.update)
    # and then also bumper. I just feel like it's appropriate to contain here as a "last resort emergency" thing
    rospy.Subscriber("/sim/bumper", Bool, ch.bumperCallback)

    # Let ROS take position of this thread until a ROS wants to kill
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass