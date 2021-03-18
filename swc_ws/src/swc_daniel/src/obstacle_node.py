#!/usr/bin/env python

import rospy
import ControlHandler
from swc_msgs.msg import Control
from swc_msgs.msg import State

_obstacle_pub = None

def main():
    global _control_pub
    global obstacleHandler

    # Initalize our node in ROS
    rospy.init_node("obstacle_node")

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _obstacle_pub = rospy.Publisher("/daniel/obstacle", Obstacle, queue_size=2)

    obstacleHandler = ObstacleHandler.ObstacleHandler()

    # Create a timer that calls timer_callback() with a period of 0.1
    rospy.Timer(rospy.Duration(0.1), publish)
    
    # get sensor data
    rospy.Subscriber("/sim/scan", LaserScan, obstacleHandler.laserCallback)
    rospy.Subscriber("/vision/compressed", Img, obstacleHandler.visionCallback)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

def publish(event):
    # Publish the message to /daniel/state so the simulator receives it
    global _obstacle_pub # globals because all funcs and stuff
    global obstacleHandler

    _obstacle_pub.publish(obstacleHandler.getFinalMessage())

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass