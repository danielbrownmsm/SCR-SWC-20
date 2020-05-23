#!/usr/bin/env python

import rospy
import Robot as r
from swc_msgs.msg import Control
from swc_msgs.msg import Gps
from sensor_msgs.msg import Imu
from swc_msgs.srv import Waypoints

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
    # Either that, or you're Justin because I asked for a code review
    # Hi Justin! (Asking Justin > reading the docs/SO/CD) => True

    # Genetic Algo stuff, uncomment when running evolutions
    #with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/swc_daniel/src/values.txt") as f:
    #    values = f.read().replace("]", "").replace("[", "").strip().replace(" ", "").replace("'", "").replace("\"", "").split(",") # long chain that gives us list of formated stuff
    #    print(values)
    #    try:
    #        values = [x.strip() for x in values]
    #        #values = [i.strip() for i in values]
    #        values = [float(x) for x in values]
    #        print(values)
    #    except Exception:
    #        print("Error converting values to floats")
    #        print(Exception)
    
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
    robot = r.Robot(waypoints.waypoints, [1,8,175000,0.01,45,8,0.07,1e-05])

    # Create a timer that calls timer_callback() with a period of read the thing
    rospy.Timer(rospy.Duration(values[6]), timer_callback)
    rospy.Timer(rospy.Duration(0.01), timer_callback2)

    # get sensor data
    rospy.Subscriber("/sim/gps", Gps, robot.updateCoords)
    rospy.Subscriber("/sim/imu", Imu, robot.updateIMU)
    
    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass