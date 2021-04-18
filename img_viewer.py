#!/usr/bin/env python

import time, os
import cv2
import numpy as np

WYPT_LOWER = np.array([5, 80, 80]) # 10-20 gets the medium-dark stuff
WYPT_UPPER = np.array([40, 255, 255]) # 20-30 seems even better at that
WYPT_KERNEL = np.ones((2, 2), np.uint8)

OBS_LOWER = np.array([157, 107, 47])
OBS_UPPER = np.array([190, 230, 255])

WAYPOINTS = True # set false for obstacle detection
FAST = True

image_num = 1

cv2.namedWindow("original")
cv2.namedWindow("altered")
os.chdir("C:\\Users\\Brown_Family01\\Documents\\GitHub\\SCR-SWC-20\\images")

while True:
    start = time.time()
    
    original = cv2.imread("image" + str(image_num) + ".png")
    cv2.imshow("original", original)

    hsv = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)

    wypt_mask = cv2.inRange(hsv, WYPT_LOWER, WYPT_UPPER)
    wypt_mask = cv2.erode(wypt_mask, WYPT_KERNEL, iterations=1)

    gray = cv2.bitwise_and(hsv, hsv, mask=wypt_mask) # apply color mask
    hsv = cv2.bitwise_and(hsv, hsv, mask=wypt_mask)
    edges = cv2.Canny(gray, 100, 300) # get edges

    # draw a crosshair
    cv2.line(hsv, (320, 230), (320, 250), (180, 255, 200), 3)
    cv2.line(hsv, (310, 240), (330, 240), (180, 255, 200), 3)
    
    hsv = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    #gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.imshow("altered", edges)
    #print(time.time() - start)
    key = cv2.waitKey()
    if key == ord("a"):
        image_num += 1

        if image_num > 47:
            image_num = 1
    elif key == ord("d"):
        image_num -= 1
        
        if image_num < 1:
            image_num = 47
    elif key == ord("q"):
        raise SystemExit



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
