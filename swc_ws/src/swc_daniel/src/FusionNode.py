#!/usr/bin/env python

from __future__ import print_function, division
import time
from math import cos, sin, atan, radians, degrees

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from swc_msgs.msg import State, Gps, Control
from Util import latLonToXY, getYaw, dist


#TODO fix line endings, apparently they're mixed? VSCode could probably help. . .  no it can't. . . whatever
class TrustVals:
    # pretty sure sensor data becomes noisier the faster you go
    # latitude stdv = 1.843 <- meters varies, not gps lat/lon
    # longitude stdv = 2.138
    # accel stdv = 0.15
    # orientation stdv = 0.017
    # angle vel stdv = 0.017
    # vel stdv = 0.1
    # angle stdv = 0.4
    # power stdv = 0.1
    # LIDAR and camera don't have noise (I know camera doesn't, pretty sure LIDAR doesn't)
    
    def __init__(self, x=1, y=1, velocity=1, acceleration=1, angle=1, angle_velocity=1, angle_acceleration=1):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.acceleration = acceleration

        self.angle = angle
        self.angle_velocity = angle_velocity
        self.angle_acceleration = angle_acceleration


DEFAULT_TRUST = TrustVals()
#                          X     Y     Vel  Acl   Ang  A_V  Ang_Acl
GPS_TRUST = TrustVals(     2,    2,    0.5, 0.5,  0.5, 0.5, 0.5)
IMU_TRUST = TrustVals(     0.25, 0.25, 0.5, 1,    3,   3,   3)
CONTROL_TRUST = TrustVals( 0.9,  0.9,  2,   1,    0.6, 0.4, 0.2)
VELOCITY_TRUST = TrustVals(1,    1,    3,   2,    0.5, 0.5, 0.5)

class RobotState:
    """A class to hold the state of a robot including position, velocity, and
    acceleration, as well as their angular equivalents. How much you trust each
    of those values (for use in a weighted average) can be modified by passing
    a TrustVals object in for trust_vals
    """
    def __init__(self, x=0, y=0, velocity=0, acceleration=0, angle=0, angle_velocity=0, angle_acceleration=0, timestamp=0, prev_timestamp=0, trust_vals=DEFAULT_TRUST):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.acceleration = acceleration

        self.angle = angle
        self.angle_velocity = angle_velocity
        self.angle_acceleration = angle_acceleration

        self.timestamp = timestamp
        self.prev_timestamp = prev_timestamp

        self.prev_state = None

        self.trust_vals = trust_vals

    # acceleration = change velocity / change time
    # velocity = change position / change time
    
    # velocity += acceleration * time
    # position += velocity * time

    def __str__(self):
        string = ""
        string += "x: " + str(self.x) + "\n"
        string += "y: " + str(self.y) + "\n"
        string += "velocity: " + str(self.velocity) + "\n"
        #string += "acceleration: " + str(self.acceleration) + "\n\n"

        string += "angle: " + str(self.angle) + "\n"
        #string += "angle_velocity: " + str(self.angle_velocity) + "\n"
        #string += "angle_acceleration: " + str(self.angle_acceleration)

        return string


def integrate():
    # this is x += velocity * delta time
    return

def derivative():
    # this is velocity = delta x / delta time
    return

# position += velocity * d_time
# velocity = d_position / d_time
# acceleration = d_velocity / d_time
# velocity += acceleration * d_time
# which means (?) position += acceleration * d_time * d_time ???
# and (?) acceleration = d_position / d_time // d_time ?????? yeah I'm not too sure about this one
# Obviously needing to cos() and sin() these but yeah
# 

class VelocityHandler:
    """A class to handle updating a state based on the /sim/velocity topic
    """
    # for velocity you do
    #x = <trig>
    #y = <trig>
    #acceleration = (velocity - last_velocity) / delta_time
    #angle = <outside source>
    
    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0

        self.timestamp = 0
        self.hasRun = False
        self.prev_state = None

        self.trust_vals = VELOCITY_TRUST
    
    def update(self, data, angle):
        if self.hasRun:
            self.timestamp = time.time()
            delta_time = self.timestamp - self.prev_state.timestamp
            
            self.angle = angle
            self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time
        
            self.velocity = data.data
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            self.x = self.prev_state.x + self.velocity * cos(radians(self.angle)) * delta_time # trig done
            self.y = self.prev_state.y + self.velocity * sin(radians(self.angle)) * delta_time
            
            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.timestamp, self.prev_state.timestamp, VELOCITY_TRUST)
        else:
            self.prev_state = RobotState(x=0, y=0, timestamp=time.time())
            self.hasRun = True

class ControlHandler:
    """A class to handle updating a state based on the /sim/control topic
    """
    # pretty much the same as the IMU but Ackermann so blah and also terrible because it's terrible like what if you are running into an obstacle?
    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0

        self.timestamp = 0
        self.hasRun = False
        self.prev_state = None
        
        self.trust_vals = CONTROL_TRUST

    def update(self, data):
        if self.hasRun:
            self.timestamp = time.time()
            delta_time = self.timestamp - self.prev_state.timestamp
            
            # angle before position because integrated for position needs the angle for the trig
            #TODO fix because we only actual change our heading if we are going forwards so 20 deg at 1 m/s for X seconds is less than 20 deg at 4 m/s
            # yeah we need to do some forward kinematics or something
            self.angle = self.prev_state.angle# + data.turn_angle HACK disabled for now
            self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time

            self.velocity = data.speed # who wants proper terminology anyways?
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            self.x = self.prev_state.x + self.velocity * cos(radians(self.angle)) * delta_time # trig done
            self.y = self.prev_state.y + self.velocity * sin(radians(self.angle)) * delta_time
            
            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.timestamp, self.prev_state.timestamp, CONTROL_TRUST)
        else:
            self.prev_state = RobotState(x=0, y=0, timestamp=time.time())
            self.hasRun = True

class ImuHandler:
    """A class to handle updating a state based on the /sim/imu topic
    """
    # for the imu you do
    #delta_time = time - last_time
    # then
    #velocity += acceleration * delta_time
    # then
    #x, y = <trig stuff>
    # then
    #angle_velocity += angular_acceleration * delta_time
    # then
    #angle = orientation.yaw || angle += angle_velocity * delta_time
    # then
    #last_x = x
    #last_y = y
    #last_velocity = velocity
    #last_acceleration = acceleration
    # then
    #last_angle = angle
    #last_angle_velocity = last_angle
    #last_angle_acceleration = angle_acceleration
    # then
    #last_time = time

    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0

        self.timestamp = 0
        self.hasRun = False
        self.prev_state = None

        self.trust_vals = IMU_TRUST
    
    def update(self, data):
        if self.hasRun:
            self.timestamp = time.time()
            delta_time = self.timestamp - self.prev_state.timestamp
            
            # angle before position because integrated for position needs the angle for the trig
            poss_angle_o = getYaw(data.orientation)
            poss_angle_velocity_o = (self.angle - self.prev_state.angle) / delta_time

            poss_angle_velocity_av = data.angular_velocity.z
            poss_angle_av = self.prev_state.angle + poss_angle_velocity_av * delta_time

            self.angle = (poss_angle_o + poss_angle_av) / 2 # TODO get better fusing
            self.angle_velocity = (poss_angle_velocity_o + poss_angle_velocity_av) / 2
            
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time

            self.acceleration = data.linear_acceleration.x # probably x, right?
            self.velocity = self.prev_state.velocity + self.acceleration * delta_time # yeah that's right
            self.x = self.prev_state.x + self.velocity * cos(radians(self.angle)) * delta_time # trig done
            self.y = self.prev_state.y + self.velocity * sin(radians(self.angle)) * delta_time

            # SOH CAH TOA
            # 0, 0 -> 0, 100 is forwards
            # +angle is right, -angle is left
            # x is adj, y is opp
            # cos(self.angle) = x / hyp -> cos(angle) * hyp = x
            # sin(self.angle) = y / hyp

            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.timestamp, self.prev_state.timestamp, IMU_TRUST)
        else:
            self.prev_state = RobotState(x=0, y=0, timestamp=time.time())
            self.hasRun = True
        
class GpsHandler:
    """A class to handle updating a state based on the /sim/gps topic
    """
    #delta_time = time - last_time
    #x, y = latLonToXY(gps.lat, gps.lon)
    #velocity = dist(x, y, last_x, last_y) / delta_time
    #acceleration = (velocity - last_velocity) / delta_time
    #angle = atan((y - last_y) / (x - last_x))
    #angle_velocity = (self.angle - last_angle) / delta_time
    #angle_acceleration = (self.angle_velocity - last_angle_velocity) / delta_time
    #last_x = x
    #last_y = y
    #last_velocity = velocity
    #last_acceleration = acceleration
    #last_angle = angle
    #last_angle_velocity = last_angle
    #last_angle_acceleration = angle_acceleration
    #last_time = time

    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0

        self.timestamp = 0
        self.hasRun = False
        self.prev_state = None

        self.trust_vals = GPS_TRUST

    def update(self, data):
        if self.hasRun: # if we've already had a state to compare to
            self.timestamp = time.time()
            delta_time = self.timestamp - self.prev_state.timestamp

            if delta_time <= 0: # this shouldn't happen and will lead to bad
                # so run, run away Simba, and never
                return

            self.x, self.y = latLonToXY(data.latitude, data.longitude)
            self.velocity = dist(self.x, self.y, self.prev_state.x, self.prev_state.y) / delta_time
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            
            try:
                self.angle = degrees(atan((self.y - self.prev_state.y) / (self.x - self.prev_state.x)))
            except ZeroDivisionError:                
                self.angle = 0
            
            try:
                self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            except ZeroDivisionError:
                self.angle_velocity = 0
            
            try:
                self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time
            except ZeroDivisionError:
                self.angle_acceleration = 0
    
            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.timestamp, self.prev_state.timestamp, GPS_TRUST)
        else:
            self.prev_state = RobotState(x=0, y=0, timestamp=time.time())
            self.hasRun = True
        
class FusedState: #TODO wait why is this here this is just RobotState what the actual heck was I doing when writing this code?
    """A class fuse all of the sensors to get a pretty good idea of where we are at"""
    # first-order/top-level/whatever data:
    # x, y <- GPS
    # velocity <- velocity, control
    # acceleration <- IMU
    # angle <- control, IMU
    # angle_vel <- IMU
    # angle_acceleration <- IMU

    # so when propogating (as I'm now calling it)
    # solve for x,y with GPS
    # propogate all other values with that information
    # solve for angle and acceleration and angle_acceleration and all that good jazz with IMU
    # propogate all other values with that information
    # solve for angle with control
    # propogate
    # solve for velocity with velocity and control
    # propogate

    # now have a second update function that goes back through and fixes things?
    # wait, no. Predict where we will be based on dead-reckoning, then weighted average the sensors and the prediction or something
    # IDK at this point

    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0

        #trusts (for weighted average
        self.trust_x = 0
        self.trust_y = 0
        self.trust_velocity = 0
        self.trust_acceleration = 0

        self.trust_angle = 0
        self.trust_angle_velocity = 0
        self.trust_angle_acceleration = 0
    
    # does a weighted average based on how much you trust the data from each state based on stuff like standard deviation for that and stuff TODO test yeah whatevver
    def update(self, *args):
        for state in args:
            self.x += state.x * state.trust_vals.x
            self.y += state.y * state.trust_vals.y
            self.velocity += state.velocity * state.trust_vals.velocity
            self.acceleration += state.acceleration * state.trust_vals.acceleration

            self.angle += state.angle * state.trust_vals.angle
            self.angle_velocity += state.angle_velocity * state.trust_vals.angle_velocity
            self.angle_acceleration += state.angle_acceleration * state.trust_vals.angle_acceleration

            self.trust_x += state.trust_vals.x
            self.trust_y += state.trust_vals.y
            self.trust_velocity += state.trust_vals.velocity
            self.trust_acceleration += state.trust_vals.acceleration

            self.trust_angle += state.trust_vals.angle
            self.trust_angle_velocity += state.trust_vals.angle_velocity
            self.trust_angle_acceleration += state.trust_vals.angle_acceleration

        self.x /= self.trust_x
        self.y /= self.trust_y
        self.velocity /= self.trust_velocity
        self.acceleration /= self.trust_acceleration

        self.angle /= self.trust_angle
        self.angle_velocity /= self.trust_angle_velocity
        self.angle_acceleration /= self.trust_angle_acceleration
    
    def __str__(self):
        string = ""
        string += "x: " + str(self.x) + "\n"
        string += "y: " + str(self.y) + "\n"
        string += "velocity: " + str(self.velocity) + "\n"
        #string += "acceleration: " + str(self.acceleration) + "\n\n"

        string += "angle: " + str(self.angle) + "\n"
        #string += "angle_velocity: " + str(self.angle_velocity) + "\n"
        #string += "angle_acceleration: " + str(self.angle_acceleration)

        return string

class LocHandler:
    """A class to handle callbacks for all the wait actually I don't really need this?!?"""
    def __init__(self):
        self.imu_state = ImuHandler()
        self.gps_state = GpsHandler()
        self.vel_state = VelocityHandler()
        self.ctrl_state = ControlHandler()
        #self.lidar_state = LaserHandler()
        #self.vision_state = VisionHandler()
    
        self.fused_state = FusedState()

    def gpsCallback(self, data):
        self.gps_state.update(data)
    
    def imuCallback(self, data):
        self.imu_state.update(data)

    def controlCallback(self, data):
        self.ctrl_state.update(data)

    def velocityCallback(self, data):
        self.vel_state.update(data, self.imu_state.angle) #TODO do something better than this

    def lidarCallback(self, data):
        self.lidar_state.update(data)

    def visionCallback(self, data):
        self.vision_state.update(data)
    
    def getState(self):
        #TODO maybe update all the other states to match the fused state? otherwise the numbers may diverge and error becomes exponential instead of linear
        self.fused_state.update(self.gps_state, self.imu_state, self.vel_state, self.ctrl_state)
        
        # create the message
        state = State()
        state.x = self.fused_state.x
        state.y = self.fused_state.y
        state.velocity = self.fused_state.velocity
        state.acceleration = self.fused_state.acceleration

        state.angle = self.fused_state.angle
        state.angle_velocity = self.fused_state.angle_velocity
        state.angle_acceleration = self.fused_state.angle_acceleration

        return state

locHandler = LocHandler()
publisher = None
ticks = 0

def main():
    global locHandler
    global publisher

    # Initalize our node in ROS
    rospy.init_node("fusion_node")
    print("Fusion node initialized!")

    # Create a Publisher that we can use to publish messages to the /daniel/state topic
    publisher = rospy.Publisher("/daniel/state", State, queue_size=1)

    # get sensor data
    rospy.Subscriber("/sim/gps", Gps, locHandler.gpsCallback)
    rospy.Subscriber("/sim/imu", Imu, locHandler.imuCallback)
    rospy.Subscriber("/sim/velocity", Float32, locHandler.velocityCallback)
    rospy.Subscriber("/sim/control", Control, locHandler.controlCallback)

    # Create a timer that calls timer_callback() with a period of 0.1, because most of our sensors update at 10 Hz
    rospy.Timer(rospy.Duration(0.1), publish)

    print("Localization node setup complete")

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

def publish(event):
    # Publish the message to /daniel/state so the simulator receives it
    global publisher
    global locHandler
    global ticks

    _state = locHandler.getState()
    if ticks % 10 == 0:
        print(locHandler.ctrl_state.prev_state)
        print()
    ticks += 1
    publisher.publish(_state)


# so if anything imports our code (documentation tools, linters, etc.) it isn't run automatically and things don't get broken
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#Problems:
#   - GPS angle stuff seems really messed up
#   - IMU isn't doing right angle either, maybe it's returning RAD instead of DEG?
#   - IMU seems to be whack with velocity, thinking rest is -2 and max is 4/-11, maybe not initialized right?
#   - velocity works nice but is screwed over by IMU angle so pos is dead-wrong
#   - control untested, but is also probably ABSOLUTE TRASH as well