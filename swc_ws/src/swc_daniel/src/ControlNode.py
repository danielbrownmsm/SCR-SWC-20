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
        self.distancePID = PIDController(0.1, 0, 0.01) #TODO tune
        self.anglePID = PIDController(0.001, 0, 0.001) #TODO tune
        
        # skip the first because it's just the starting pos
        self.points = [latLonToXY(point.latitude, point.longitude) for point in points[1:]]
        self.points = [(-point[0], point[1]) for point in self.points]
        self.state = None
        self.targetIndex = 0
        self.goal = self.points[0]
        self.target_angle = 0
        self.hasRun = False

    def stateCallback(self, data):
        """A callback for data published to /daniel/state about the robot's state"""

        self.state = data

    def getMessage(self):
        if self.state == None: # if we haven't started getting states of something weird happens
            msg = Control()
            msg.speed = 0
            msg.turn_angle = 0
            return msg # DO NOTHING. GO NOWHERE. STAY STILL. DON'T MOVE.

        """Gets the actual control message"""
        if self.distancePID.atSetpoint() and self.hasRun:
            self.targetIndex += 1
        self.goal = self.points[self.targetIndex]
        self.target_angle = atan((self.state.x - self.goal[0]) / (self.state.y - self.goal[1]))
        #print(target_angle)
        self.anglePID.setSetpoint(degrees(self.target_angle))
        self.distancePID.setSetpoint(0)

        print(degrees(self.target_angle))

        msg = Control()

        msg.speed = self.distancePID.calculate(-dist(self.state.x, self.state.y, *self.goal))
        msg.turn_angle = clamp(self.anglePID.calculate(self.state.angle), -5, 5)

        self.hasRun = True
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

