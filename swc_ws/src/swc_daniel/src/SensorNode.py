#!/usr/bin/env python

from __future__ import print_function, division
#import time
from math import cos, sin, radians, degrees

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from swc_msgs.msg import State, Gps, Control
from Util import latLonToXY, getYaw#, dist

class RobotState(object):
    """
    A class to hold the state of a robot. Pretty much just a dict, but with easier access (no square brackets).
    A robot's state can be described by its position (x, y), velocity (straight-line relative to robot, probably x-axis?),
    acceleration (like velocity), angle (heading, rot around z axis), angle_velocity, angle_acceleration (although I'm debating
    dropping this), and a timestamp so you can compare two states and do stuff like derivative or whatever. I didn't take calculus.
    """
    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0

        self.timestamp = 0
    
    def __str__(self):
        #TODO write __str__
        return

class SensorHandler(object):
    """A class to hold the robot's state and update it with callbacks, then publish the resultant state to daniel/state
    """
    # latitude stdv = 1.843 <- meters varies, not gps lat/lon
    # longitude stdv = 2.138
    # accel stdv = 0.15
    # orientation stdv = 0.017
    # angle vel stdv = 0.017
    # vel stdv = 0.1
    # angle stdv = 0.4
    # power stdv = 0.1

    def __init__(self):
        self.state = RobotState()
        self.predicted_state = RobotState()
        self.prev_state = RobotState()

    def gpsCallback(self, data):
        """A callback for the gps. Uses latLonToXY from Util.py and updates position (x,y)"""
        x, y = latLonToXY(data.latitude, data.longitude)
        self.state.x = -x
        self.state.y = y

    def imuCallback(self, data):
        """
        A callback for the imu. Uses getYaw from Util.py to convert a quat to a euler.
        Updates angle, angle velocity, and acceleration (x axis or whatever points from the back to the front
        """
        self.state.acceleration = data.linear_acceleration.x #TODO prob right axis check
        self.state.angle = degrees(getYaw(data.orientation))
        self.state.angle_velocity = data.angular_velocity.z #TODO yeah that's probably not right way to get it check

    def controlCallback(self, data):
        """A callback for our control messages that we output from ControlNode. Basically dead-reckoning"""
        velocity = data.speed
        angle = self.prev_state.angle + data.turn_angle

        self.predicted_state.x = sin(radians(angle)) * velocity
        self.predicted_state.y = cos(radians(angle)) * velocity
        self.predicted_state.velocity = velocity
        #self.predicted_state.acceleration

        self.predicted_state.angle = angle
        #self.predicted_state.angle_velocity
        #self.predicted_state.angle_acceleration

    def velocityCallback(self, data):
        """A callback for the velocity publisher because it has sweet, sweet 0.1 standard deviation"""
        self.state.velocity = data.data

    def propogate(self):
        #TODO write
        # this is where we would average and weight everything and solve remaining variables and cross solve things and stuff
        # and also use predicted state somehow idk tho. Justin K said to use an Extended Kalman but idk how to like
        # let me take calculus and I'll get back to you like what the heck is a Jacobian and how do I use it and
        # why am I 'linearizing' and even is what that and why am i here how did i get here i don't know hey random internet
        # stranger reading this comment for some reason why are _you_ here? A duck walked up to a lemonade stand and somebody
        # once told me the world was gonna roll me I took the midnight train going anywhere oh mama mia mama mia let me go
        # if it hadn't been for cotton-eyed joe, cha cha real smooth go to work "to the left" * 4
        pass

    def getMessage(self):
        """Gets a State message for publishing"""
        self.propogate() # maek sho we usin the finest stuf

        msg = State()
        msg.x = self.state.x
        msg.y = self.state.y
        msg.velocity = self.state.velocity
        msg.acceleration = self.state.acceleration

        msg.angle = self.state.angle
        msg.angle_velocity = self.state.angle_velocity
        msg.angle_acceleration = self.state.angle_acceleration

        self.prev_state = self.state
        return msg

def publish(event):
    """Publishes the robot's State to /daniel/state"""
    global sensorHandler
    global publisher

    publisher.publish(sensorHandler.getMessage())

def printState(event):
    global sensorHandler
#    print()
#    print(sensorHandler.getMessage())
    pass

if __name__ == "__main__":
    try:
        # Initalize our node in ROS
        rospy.init_node("sensor_node")
        rospy.logwarn("Sensor node initialized!")

        publisher = rospy.Publisher("/daniel/state", State, queue_size=1)

        # get sensor data
        sensorHandler = SensorHandler()
        rospy.Subscriber("/sim/gps", Gps, sensorHandler.gpsCallback)
        rospy.Subscriber("/sim/imu", Imu, sensorHandler.imuCallback)
        rospy.Subscriber("/sim/velocity", Float32, sensorHandler.velocityCallback)
        rospy.Subscriber("/sim/control", Control, sensorHandler.controlCallback)

        # Create a timer that calls timer_callback() with a period of 0.1, because most of our sensors update at 10 Hz
        rospy.Timer(rospy.Duration(0.1), publish)
        rospy.Timer(rospy.Duration(1), printState)

        # Let ROS take control of this thread until a ROS wants to kill
        rospy.logwarn("Sensor node setup complete")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
