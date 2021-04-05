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
        self.ticks = 0

    def visionCallback(self, data):
        self.ticks += 1
        if not self.hasRun and self.ticks % 100 == 0:
            #print(data)
            arr = np.fromstring(data.data, np.uint8)
            img_np = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            #print(img_np)
            hsv = cv2.cvtColor(img_np, cv2.COLOR_BGR2HSV)
            #hsv = cv2.GaussianBlur(hsv_, (5, 5), 0)
            #print(hsv)
            
            # UPDATE: this has been tuned by a color/filter script thingy and should be ~best
            lower = np.array([5, 80, 80]) # 10-20 gets the medium-dark stuff
            upper = np.array([30, 225, 225]) # 20-30 seems even better at that
            # 30-40 gets very little, 0-10 gets nothing
            # nothing seems to get the bright stuff
            #print("We made it this far")

            mask = cv2.inRange(hsv, lower, upper)
            #print(mask)
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # ???
            #print(contours)
            contours = contours[0] if len(contours) == 2 else contours[1]

            for contour in contours:
                color = (255, 0, 0)
                x, y, width, height = cv2.boundingRect(contour)
                if width > height * 2.5: # filter out really wide short ones
                    color = (255, 0, 255)
                    continue
                if width <= 7 or height <= 7: # filter out small ones
                    color = (255, 0, 255)
                    continue
                if y < 100:
                    continue
                approx = cv2.approxPolyDP(contour, 0.04 * cv2.arcLength(contour, True), True)
                print(len(approx))
                #if len(approx) != 3:
                #    continue

                #print(y)
                cv2.rectangle(img_np, (x, y), (x + width, y + height), color, 1)
                #print(x, y, width, height)
                #print()
            #print(len(contours))

            os.chdir("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20")
            cv2.imwrite("image.png", img_np)
            print("image wrote")
            
            #self.hasRun = True
            #raise SystemExit

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
