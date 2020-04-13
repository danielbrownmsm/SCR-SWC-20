#!/usr/bin/env python

import rospy
import random
import Robot as r
from swc_msgs.msg import Control
from swc_msgs.msg import Gps
from sensor_msgs.msg import Imu
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from swc_msgs.srv import Waypoints

_control_pub = None

def timer_callback(event):
    # Publish the message to /sim/control so the simulator receives it
    _control_pub.publish(robot.getAction())

def main():
    global _control_pub
    global robot # yes, bad practice. Too bad. deal with it. After all, you're most likely me.
    # Either that, or you're Justin because I asked for a code review
    # Hi Justin! (Asking Justin > reading the docs/SO/CD) => True
    
    # Initalize our node in ROS
    rospy.init_node('py_robot_control_node')

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service('/sim/waypoints')
    waypoints = rospy.ServiceProxy('/sim/waypoints', Waypoints)()

    # Define where we need to go (order is: start, bonus, bonus, bonus, finish with bonusses roughly in 
    # order of how far away they are)
    # create instance of Robot class
    robot = r.Robot(waypoints.waypoints)

    # Create a timer that calls timer_callback() with a period of read the thing
    rospy.Timer(rospy.Duration(0.07), timer_callback)

    # get sensor data
    rospy.Subscriber("/sim/gps", Gps, robot.updateCoords)
    rospy.Subscriber("/sim/imu", Imu, robot.updateIMU)
    #rospy.Subscriber("/sim/image/compressed", CompressedImage, robot.updateCamera)
    #rospy.Subscriber("/scan", LaserScan, robot.updateLaser)
    rospy.Subscriber("/sim/bumper", Bool, robot.updateBumper)
    #rospy.Subscriber("/velocity", Velocity, robot.updateVelocity)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass