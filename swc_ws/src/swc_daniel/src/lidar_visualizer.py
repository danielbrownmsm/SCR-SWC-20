#!/usr/bin/env python

import rospy
from sensor_msgs import LaserScan

def callback(data):
    for cell in data.ranges:
        pass
    print(data.time_increment) # interpolation or something?

def main():
    # Initalize our node in ROS
    rospy.init_node("py_lidar_viz_node")

    # get sensor data
    rospy.Subscriber("/scan", Laser, callback)
    
    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass