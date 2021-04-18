#!/usr/bin/env python

from __future__ import print_function, division

import rospy, os, time
from sensor_msgs.msg import CompressedImage
from swc_msgs.msg import Vision

import numpy as np
import cv2

WYPT_LOWER = np.array([5, 80, 80]) # 10-20 gets the medium-dark stuff
WYPT_UPPER = np.array([40, 255, 255]) # 20-30 seems even better at that
OBS_LOWER = np.array([157, 107, 47])
OBS_UPPER = np.array([190, 230, 255])
KERNEL = np.ones((2, 2), np.uint8)
FAST = True

class VisionHandler(object):
    def __init__(self):
        self.x_offset = 0
        self.distance = 0
        self.detected = False

        self.ticks = 0
        self.img_cnt = 0

    def visionCallback(self, data):
        # to limit our CPU usage because my machine is slow
        self.ticks += 1

        if self.ticks % 180 == 0:
            start = time.time()
            self.img_cnt += 1

            arr = np.fromstring(data.data, np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale

            wypt_mask = cv2.inRange(hsv_img, WYPT_LOWER, WYPT_UPPER)
            wypt_mask = cv2.erode(wypt_mask, KERNEL, iterations=1)
            
            gray_img = cv2.bitwise_and(hsv_img, hsv_img, mask=wypt_mask) # apply color mask
            edges = cv2.Canny(gray_img, 100, 200) # get edges

            wypt_contours = cv2.findContours(wypt_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # ???
            wypt_contours = wypt_contours[0] if len(wypt_contours) == 2 else wypt_contours[1]

            largest = {
                "width":0,
                "height":0,
                "x":0,
                "y":0
            }

            if not FAST: # draw boxes and all that for testing and debugging
                output_img = cv2.drawContours(img, wypt_contours, -1, (255, 0, 255), 4)
                for contour in wypt_contours:
                    color = (255, 0, 0)
                    x, y, width, height = cv2.boundingRect(contour)
                    #if 0.3 < width / height > 4: # filter out ones with weird aspect ratios
                    #    color = (100, 0, 255)
                    #elif width <= 7 or height <= 7: # filter out small ones
                    #    color = (255, 255, 0)
                    #    continue
                    #elif y > 250: # reject low ones (ie the ground)
                    #    color = (0, 0, 255)
                    #elif cv2.contourArea(contour) * 4 < width * height: # triangles have ~1/2 area of their bounding rect
                    #    color = (0, 0, 0)
                    cv2.rectangle(output_img, (x, y), (x + width, y + height), color, 2)
            else:
                for contour in wypt_contours:
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

            output_img = edges
            os.chdir("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/images")
            cv2.imwrite("image" + str(self.img_cnt) + ".png", img)
            #cv2.imwrite("image_orig" + str(self.img_cnt) + ".png", img)
            #cv2.imwrite("image_edge" + str(self.img_cnt) + ".png", edges)
            #cv2.imwrite("image_gray" + str(self.img_cnt) + ".png", gray_img)
            #cv2.imwrite("image_out" + str(self.img_cnt) + ".png", output_img)

            print(self.img_cnt)
            
            print(time.time() - start)

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
