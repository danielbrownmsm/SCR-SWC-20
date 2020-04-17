#!/usr/bin/env python

import rospy
from swc_msgs.msg import Gps
from swc_msgs.msg import Position
from sensor_msgs.msg import Imu

position_pub = None

def updateCoords(self, data):
    

def timer_callback(event):
    position_pub.publish(0)

def main():
    global position_pub
    # Initalize our node in ROS
    rospy.init_node('position_node')
    # Create a Publisher that we can use to publish messages to the /sim/control topic
    position_pub = rospy.Publisher("/position", Float32, queue_size=1)
    # Create a timer that calls timer_callback() with a period of 0.07
    rospy.Timer(rospy.Duration(0.07), timer_callback)

    # get sensor data
    rospy.Subscriber("/sim/gps", Gps, robot.updateCoords)
    rospy.Subscriber("/sim/imu", Imu, robot.updateIMU)
    #rospy.Subscriber("/scan", LaserScan, robot.updateLaser)
    rospy.Subscriber("/sim/velocity", Float32, robot.updateVelocity)

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass