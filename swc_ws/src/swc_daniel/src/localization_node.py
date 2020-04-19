#!/usr/bin/env python

import rospy
import LocalizationHandler as LH
from swc_msgs.msg import Control
from swc_msgs.msg import Gps
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from swc_msgs.srv import Waypoints

_position_pub = None

def timer_callback(event):
    # Publish the message to /sim/position
    _position_pub.publish(lh.getMessage())

def main():
    global _position_pub
    global lh

    # Initalize our node in ROS
    rospy.init_node('localization_node.py')

    # Create a Publisher that we can use to publish messages to the /sim/position topic
    _position_pub = rospy.Publisher("/sim/position", Position, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()

    # Define where we need to go (order is: start, bonus, bonus, bonus, finish with bonuses roughly in 
    # order of how far away they are)
    lh = LH.LocalizationHandler(waypoints.waypoints)

    # Create a timer that calls timer_callback() with a period of 0.07
    rospy.Timer(rospy.Duration(0.07), timer_callback)

    # get sensor data
    rospy.Subscriber("/sim/gps", Gps, lh.gpsCallback)
    rospy.Subscriber("/sim/control", Control, lh.controlCallback)
    rospy.Subscriber("/sim/imu", Imu, lh.imuCallback)
    rospy.Subscriber("/sim/velocity", Float32, lh.velocityCallback)

    # Let ROS take position of this thread until a ROS wants to kill
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass