#!/usr/bin/env python

from __future__ import print_function, division
import time, os, rospy

# basically just sets max time to 40s or whatever you want. Just change the number in the sleep()
if __name__ == "__main__":
    try:
        # Initalize our node in ROS
        rospy.init_node("timer_node")
        rospy.logwarn("Timer node initialized!")
        time.sleep(35)
        raise SystemExit
        
    except rospy.ROSInterruptException:
        pass

