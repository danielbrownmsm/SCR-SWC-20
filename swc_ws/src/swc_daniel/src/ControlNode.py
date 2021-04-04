#!/usr/bin/env python

from __future__ import print_function, division
from math import degrees, atan

import rospy
from Util import PIDController, dist, latLonToXY, clamp
from swc_msgs.msg import State, Control
from swc_msgs.srv import Waypoints

class ControlHandler(object):
    """A class to handle controlling our robot"""
    def __init__(self, points):
        # read our GA tunable parameters
        with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/swc_daniel/src/vals.txt", "r") as f:
            vals = f.readlines()
            for index, val in enumerate(vals):
                vals[index] = float(val.strip())

        # init PIDS        
        self.distancePID = PIDController(vals[0], 0, vals[1])
        self.distancePID.setSetpoint(0)
        self.distancePID.threshold = vals[2]
        self.distancePID.velocityThreshold = vals[3]
        
        self.anglePID = PIDController(0.05, 0, 0.001)
        self.anglePID.setSetpoint(0)
        
        # skip the first because it's just the starting pos
        self.points = [latLonToXY(point.latitude, point.longitude) for point in points[1:]]
        self.points = [(-point[0], point[1]) for point in self.points]
        self.goal = self.points[0]
        self.targetIndex = 0
        
        self.state = None
        self.target_angle = 0
        self.hasRun = False
        
    def stateCallback(self, data):
        """A callback for data published to /daniel/state about the robot's state"""
        self.state = data

    def getMessage(self):
        """Gets the actual control message"""
        if self.state == None: # if we haven't started getting states or something weird happens
            msg = Control()
            msg.speed = 0
            msg.turn_angle = 0
            return msg # DO NOTHING. GO NOWHERE. STAY STILL. DON'T MOVE.
        
        # if we've driven far enough
        if self.distancePID.atSetpoint() and self.hasRun and self.targetIndex < 3:
            self.targetIndex += 1 # go for next waypoint
            rospy.logwarn("Going for next waypoint")

        self.goal = self.points[self.targetIndex] # assign goal
        self.target_angle = -degrees(atan((self.state.x - self.goal[0])  / (self.state.y - self.goal[1]))) # get target

        msg = Control()
        msg.speed = self.distancePID.calculate(-dist(self.state.x, self.state.y, *self.goal)) # -dist because we are driving it to 0
        # which means if dist were positive this would output a negative, driving us backwards and increasing error
        msg.turn_angle = clamp(self.anglePID.calculate(self.target_angle - self.state.angle), -10, 10) # clamp output

        if self.state.y > self.goal[1] + 0.5: # if we're past it (for sure)
            msg.speed *= -1 # then go backwards
            msg.turn_angle *= -1 # which means turns are inverted

        self.hasRun = True # so we don't prematurely go for another waypoint
        return msg

def publish(event):
    # Publish the message to /daniel/control so the simulator receives it
    global publisher
    global controlHandler

    publisher.publish(controlHandler.getMessage())

if __name__ == "__main__":
    try:
        # Initalize our node in ROS
        rospy.init_node("control_node")
        rospy.logwarn("Control node initialized!")

        # Create a Publisher that we can use to publish messages to the /daniel/control topic
        publisher = rospy.Publisher("/sim/control", Control, queue_size=1)

        # Wait for Waypoints service and then request waypoints
        rospy.wait_for_service("/sim/waypoints")
        waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()
        rospy.logdebug("Waypoints aquired!")

        controlHandler = ControlHandler(waypoints.waypoints)
        for point in controlHandler.points:
            x, y = point
            rospy.logwarn((round(x, 3), round(y, 3)))

        # subscribe to our state topic
        rospy.Subscriber("/daniel/state", State, controlHandler.stateCallback)

        rospy.Timer(rospy.Duration(0.1), publish)

        # Let ROS take control of this thread until a ROS wants to kill
        rospy.logwarn("Control node setup complete")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
