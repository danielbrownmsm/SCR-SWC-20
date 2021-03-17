#!/usr/bin/env python

import rospy
import LocHandler
from swc_msgs.msg import Gps
from swc_msgs.msg import State
from sensor_msgs.msg import Imu
from swc_msgs.srv import Waypoints
from swc_msgs.msg import Control
from std_msgs.msg import Float32

_localization_pub = None

def main():
    global _localization_pub
    global locHandler

    # Initalize our node in ROS
    rospy.init_node("localization_node")

    # Create a Publisher that we can use to publish messages to the /daniel/state topic
    _localization_pub = rospy.Publisher("/daniel/state", State, queue_size=3)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service("/sim/waypoints")
    waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()

    print("Waypoints aquired!")
    locHandler = LocHandler.LocHandler(waypoints.waypoints[0])

    # Create a timer that calls timer_callback() with a period of 0.1
    rospy.Timer(rospy.Duration(0.1), publish)
    
    # get sensor data
    rospy.Subscriber("/sim/gps", Gps, locHandler.gpsCallback)
    rospy.Subscriber("/sim/imu", Imu, locHandler.imuCallback)
    rospy.Subscriber("/sim/velocity", Float32, locHandler.velocityCallback)
    rospy.Subscriber("/sim/control", Control, locHandler.controlCallback)

    print("Subscribed!")

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

def publish(event):
    # Publish the message to /daniel/state so the simulator receives it
    global _localization_pub # globals because all funcs and stuff
    global locHandler

    _localization_pub.publish(locHandler.getState())

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass