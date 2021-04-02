#!/usr/bin/env python

from __future__ import print_function, division
from math import degrees, atan

import rospy
from Util import PIDController, dist, latLonToXY
from swc_msgs.msg import State, Control
from swc_msgs.srv import Waypoints

class ControlHandler(object):
    """A class to handle controlling our robot"""
    def __init__(self, points):
        self.distancePID = PIDController(1, 0, 0.01) #TODO tune
        self.anglePID = PIDController(1, 0, 0.1) #TODO tune
        
        # skip the first because it's just the starting pos
        self.points = [latLonToXY(point.latitude, point.longitude) for point in points[1:]]
        self.state = None
        self.targetIndex = 0
        self.goal = self.points[0]
        self.target_angle = 0

    def stateCallback(self, data):
        """A callback for data published to /daniel/state about the robot's state"""
        self.state = data

    def getMessage(self):
        """Gets the actual control message"""
        if self.distancePID.atSetpoint():
            self.targetIndex += 1
        self.goal = self.points[self.targetIndex]
        self.target_angle = atan((self.state.x - self.goal[0]) / (self.state.y - self.goal[1]))
        self.anglePID.setSetpoint(degrees(self.target_angle))

        msg = Control()

        msg.speed = self.distancePID.calculate(dist(self.state.x, self.state.y, *self.goal))
        msg.turn_angle = self.anglePID.calculate(self.state.angle)

        return msg

if __name__ == "__main__":
    try:
        # Initalize our node in ROS
        rospy.init_node("control_node")
        rospy.loginfo("Control node initialized!")

        # Create a Publisher that we can use to publish messages to the /daniel/control topic
        publisher = rospy.Publisher("/sim/control", Control, queue_size=1)

        # Wait for Waypoints service and then request waypoints
        rospy.wait_for_service("/sim/waypoints")
        waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()
        rospy.loginfo("Waypoints aquired!")

        controlHandler = ControlHandler(waypoints.waypoints)

        # subscribe to our state topic
        rospy.Subscriber("/daniel/state", State, controlHandler.stateCallback)

        rospy.Timer(rospy.Duration(0.1), publish)

        # Let ROS take control of this thread until a ROS wants to kill
        rospy.loginfo("Control node setup complete")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

def publish(event):
    # Publish the message to /daniel/control so the simulator receives it
    global publisher
    global controlHandler

    publisher.publish(controlHandler.getMessage())
