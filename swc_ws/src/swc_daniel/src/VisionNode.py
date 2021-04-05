#!/usr/bin/env python

from __future__ import print_function, division

import rospy, time, os
from sensor_msgs.msg import CompressedImage

import numpy as np
import cv2

class VisionHandler(object):
    def __init__(self):
        self.waypoints = []
        self.obstacles = []
        self.hasRun = False

    def visionCallback(self, data):
        if not self.hasRun:
            #print(data)
            arr = np.fromstring(data.data, np.uint8)
            img_np = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            #print(img_np)
            hsv_ = cv2.cvtColor(img_np, cv2.COLOR_BGR2HSV)
            hsv = cv2.blur(hsv_, (5, 5))
            #print(hsv)
            
            lower = np.array([20, 100, 100])
            upper = np.array([40, 225, 225])
            print("We made it this far")

            mask = cv2.inRange(hsv, lower, upper)
            #print(mask)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # ???
            #print(contours)
            contours = contours[0] if len(contours) == 2 else contours[1]

            for contour in contours:
                x, y, width, height = cv2.boundingRect(contour)
                cv2.rectangle(img_np, (x, y), (x + width, y + height), (0, 0, 255), 1)
                #print(x, y, width, height)
                #print()
            print(len(contours))

            os.chdir("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20")
            cv2.imwrite("image.png", img_np)
            print("image wrote")
            
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
        time.sleep(8)
        rospy.Subscriber("/sim/image/compressed", CompressedImage, visionHandler.visionCallback)

        #rospy.Timer(rospy.Duration(0.1), publish)

        # Let ROS take control of this thread until a ROS wants to kill
        rospy.logwarn("Vision node setup complete")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
