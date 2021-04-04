#!/usr/bin/env python

from __future__ import print_function, division

import rospy
from sensor_msgs.msg import CompressedImage

class VisionHandler(object):
    def __init__(self):
        self.waypoints = []
        self.obstacles = []
        self.hasRun = False

    def visionCallback(self, data):
        if not self.hasRun:
            print(data)
            self.hasRun = True
            raise SystemExit

    def getMessage(self):
        msg = Vision()

        return msg

def publish(event):
    global visionHandler
    global publisher

    publisher.publish(visionHandler.getMessage())

if __name__ == "__main__":
    try:
        # Initalize our node in ROS
        rospy.init_node("vision_node")
        rospy.logwarn("Vision node initialized!")

        # Create a Publisher that we can use to publish messages to the /daniel/control topic
        #publisher = rospy.Publisher("/daniel/vision", Vision, queue_size=1)

        visionHandler = VisionHandler()

        # subscribe to our state topic
        rospy.Subscriber("/sim/image/compressed", CompressedImage, visionHandler.visionCallback)

        #rospy.Timer(rospy.Duration(0.1), publish)

        # Let ROS take control of this thread until a ROS wants to kill
        rospy.logwarn("Vision node setup complete")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
