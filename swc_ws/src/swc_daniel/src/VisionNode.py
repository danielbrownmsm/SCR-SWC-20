#!/usr/bin/env python

from __future__ import print_function, division

import rospy, os, time
from sensor_msgs.msg import CompressedImage
from swc_msgs.msg import Vision

import numpy as np
import cv2

LOWER = np.array([5, 80, 80]) # 10-20 gets the medium-dark stuff
UPPER = np.array([40, 255, 255]) # 20-30 seems even better at that
KERNEL = np.ones((2, 2), np.uint8)
FAST = True

class VisionHandler(object):
    def __init__(self):
        self.x_offset = 0
        self.distance = 0
        self.detected = False

        self.ticks = 0

    def visionCallback(self, data):
        # to limit our CPU usage because my machine is slow
        self.ticks += 1

        if self.ticks % 10 == 0:
            start = time.time()
            arr = np.fromstring(data.data, np.uint8)
            img_np = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            hsv = cv2.cvtColor(img_np, cv2.COLOR_BGR2HSV)

            mask = cv2.inRange(hsv, LOWER, UPPER)
            mask = cv2.erode(mask, KERNEL, iterations=1)

            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # ???
            contours = contours[0] if len(contours) == 2 else contours[1]

            largest = {
                "width":0,
                "height":0,
                "x":0,
                "y":0
            }

            if not FAST: # draw boxes and all that for testing and debugging
                img_np = cv2.drawContours(img_np, contours, -1, (255, 0, 255), 4)
                for contour in contours:
                    color = (255, 0, 0)
                    x, y, width, height = cv2.boundingRect(contour)
                    if 0.3 < width / height > 4: # filter out ones with weird aspect ratios
                        color = (100, 0, 255)
                    elif width <= 7 or height <= 7: # filter out small ones
                        color = (255, 255, 0)
                        continue
                    elif y > 250: # reject low ones (ie the ground)
                        color = (0, 0, 255)
                    #elif cv2.contourArea(contour) * 4 < width * height: # triangles have ~1/2 area of their bounding rect
                    #    color = (0, 0, 0)
                    #cv2.rectangle(img_np, (x, y), (x + width, y + height), color, 2)
            else:
                for contour in contours:
                    x, y, width, height = cv2.boundingRect(contour)
                    if width <= 7 or height <= 7: # filter out small ones
                        continue
                    elif 0.3 < width / height > 4: # filter out ones with weird aspect ratios
                        continue
                    elif y > 250: # reject low ones (ie the ground)
                        continue

                    if width * height > largest["width"] * largest["height"]:
                        largest["width"] = width
                        largest["height"] = height
                        largest["x"] = x
                        largest["y"] = y

            if largest["x"] != 0 and largest["width"] > 5:
                self.x_offset = largest["x"] - 300 + largest["width"] / 2
                self.distance = largest["height"]
                self.detected = True
            else:
                self.detected = False
            #os.chdir("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20")
            #cv2.imwrite("image.png", img_np)
            #print("image wrote")
            
            #print(time.time() - start)

    def getMessage(self):
        msg = Vision()
        msg.x_offset = self.x_offset
        msg.distance = self.distance
        msg.detected = self.detected

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
        publisher = rospy.Publisher("/daniel/vision", Vision, queue_size=1)
        visionHandler = VisionHandler()

        # subscribe to the camera
        rospy.Subscriber("/sim/image/compressed", CompressedImage, visionHandler.visionCallback)

        rospy.Timer(rospy.Duration(0.1), publish)

        # Let ROS take control of this thread until a ROS wants to kill
        rospy.logwarn("Vision node setup complete")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
