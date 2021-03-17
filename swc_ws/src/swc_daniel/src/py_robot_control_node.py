#!/usr/bin/env python

import rospy
import Robot as r
from ObstacleHandler import ObstacleHandler
import LocHandler
from swc_msgs.msg import Control
from swc_msgs.msg import Gps
from swc_msgs.msg import State
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from swc_msgs.srv import Waypoints
from std_msgs.msg import Float32

_control_pub = None

def timer_callback2(event):
    robot.updateTime() # yes, hacky. deal with it

def timer_callback(event):
    # Publish the message to /sim/control so the simulator receives it
    _control_pub.publish(robot.getAction())

def main():
    print("We're in")
    global _control_pub
    global robot # yes, bad practice. Too bad. deal with it. After all, you're most likely me.
    
    # Initalize our node in ROS
    rospy.init_node("py_robot_control_node")

    # Create a Publisher that we can use to publish messages to the /sim/control topic
    _control_pub = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service("/sim/waypoints")
    waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()

    # Define where we need to go (order is: start, bonus, bonus, bonus, finish with bonusses roughly in 
    # order of how far away they are)
    # create instance of Robot class
    print("Waypoints aquired!")
    # minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshol
    robot = r.Robot(waypoints.waypoints, [1,4,175000,0,5,8,0.07,1e-05])
    
    obsHandler = ObstacleHandler()
    rospy.Subscriber("/scan", LaserScan, obsHandler.laserCallback)
    rospy.Subscriber("/daniel/state", State, obsHandler.stateCallback)
    
    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass