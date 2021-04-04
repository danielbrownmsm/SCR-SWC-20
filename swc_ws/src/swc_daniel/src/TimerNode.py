#!/usr/bin/env python

from __future__ import print_function, division
import time, os, rospy


if __name__ == "__main__":
    try:
        # Initalize our node in ROS
        rospy.init_node("timer_node")
        rospy.logwarn("Timer node initialized!")
        time.sleep(40)
        raise SystemExit
        
    except rospy.ROSInterruptException:
        pass

