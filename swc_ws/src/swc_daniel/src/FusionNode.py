#!/usr/bin/env python

import time
from math import cos, sin, atan2, atan

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from swc_msgs.msg import State, Gps, Control
from swc_msgs.srv import Waypoints

from Util import latLonToXY, getYaw, dist

#TODO either use or remove
DEFAULT_TRUST = {
    "x":1,
    "y":1,
    "velocity":1,
    "accel":1,
    "angle":1,
    "angle_velocity":1,
    "angle_accel":1
}

class RobotState:
    def __init__(self, x=0, y=0, velocity=0, acceleration=0, angle=0, angle_velocity=0, angle_acceleration=0, time=0, prev_time=0, trust_vals=DEFAULT_TRUST):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.acceleration = acceleration

        self.angle = angle
        self.angle_velocity = angle_velocity
        self.angle_acceleration = angle_acceleration

        self.time = time
        self.prev_time = prev_time

        self.prev_state = None

    # acceleration = change velocity / change time
    # velocity = change position / change time
    
    # velocity += acceleration * time
    # position += velocity * time

    # always work with the hypot if you have it because going from dist -> x, y adds error

    def solveState(self):
        pass
    
    def update(self):
        pass

# =======================================================================================================================
# TODO WAIT NO MAKE AN INIT_STATE METHOD OF ALL THE HANDLERS THAT IS CALLED AT THE BEGINNING SO THEY KNOW THEIR POSITIONS (starting waypoints!) AND CAN GET THE PREV_STATE VAR INITED AND STUFF DANIEL YOU'RE A GENIUS (occasionaly)
# TODO BUG FIXME XXX HACK the current else: State(x=-37) is wrong b/c we should treat it like (0, 0) and so also GPS callback is wrong so need to like +37 or something
#TODO FIXME XXX BUG HACK wrap all these cos and sins in degrees() calls because that's what we all expect
# =======================================================================================================================

class VelocityHandler:
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

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data, angle):
        if self.hasRun:
            self.time = time.time()
            delta_time = self.time - self.prev_state.time
            
            self.angle = angle
            self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time
        
            self.velocity = data.data
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            self.x = self.prev_state.x + self.velocity * cos(self.angle) * delta_time # trig done
            self.y = self.prev_state.y + self.velocity * sin(self.angle) * delta_time
            
            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else:
            self.prev_state = RobotState(x=-37, y=0, time=time.time())
            self.hasRun = True

class ControlHandler:
    # pretty much the same as the IMU but Ackermann so blah and also terrible because it's terrible like what if you are running into an obstacle?
    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data):
        if self.hasRun:
            self.time = time.time()
            delta_time = self.time - self.prev_state.time
            
            # angle before position because integrated for position needs the angle for the trig
            #TODO fix because we only actual change our heading if we are going forwards so 20 deg at 1 m/s for X seconds is less than 20 deg at 4 m/s
            # yeah we need to do some forward kinematics or something
            self.angle = self.prev_state.angle + data.turn_angle
            self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time

            self.velocity = data.speed # who wants proper terminology anyways?
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            self.x = self.prev_state.x + self.velocity * cos(self.angle) * delta_time # trig done
            self.y = self.prev_state.y + self.velocity * sin(self.angle) * delta_time
            
            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else:
            self.prev_state = RobotState(x=-37, y=0, time=time.time())
            self.hasRun = True

class ImuHandler:
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

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data):
        if self.hasRun:
            self.time = time.time()
            delta_time = self.time - self.prev_state.time
            
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
            self.x = self.prev_state.x + self.velocity * cos(self.angle) * delta_time # trig done
            self.y = self.prev_state.y + self.velocity * sin(self.angle) * delta_time

            # SOH CAH TOA
            # 0, 0 -> 0, 100 is forwards
            # +angle is right, -angle is left
            # x is adj, y is opp
            # cos(self.angle) = x / hyp -> cos(angle) * hyp = x
            # sin(self.angle) = y / hyp

            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else:
            self.prev_state = RobotState(x=-37, y=0, time=time.time())
            self.hasRun = True
        
class GpsHandler:
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

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data):
        if self.hasRun: # if we've already had a state to compare to
            self.time = time.time()
            delta_time = self.time - self.prev_state.time

            if delta_time <= 0: # this shouldn't happen and will lead to bad
                # so run, run away Simba, and never
                return

            self.x, self.y = latLonToXY(data.latitude, data.longitude)
            self.velocity = dist(self.x, self.y, self.prev_state.x, self.prev_state.y) / delta_time
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            
            try:
                self.angle = atan((self.y - self.prev_state.y) / (self.x - self.prev_state.x))
            except ZeroDivisionError as e:
                print("zero division thingy happened")
                self.angle = 0
            
            try:
                self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            except ZeroDivisionError:
                print("zero division thingy happened")
                self.angle_velocity = 0
            
            try:
                self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time
            except ZeroDivisionError:
                print("zero division thingy happened")
                self.angle_acceleration = 0
    
            self.prev_state = RobotState(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else:
            self.prev_state = RobotState(x=-37, y=0, time=time.time())
            self.hasRun = True
        
class FusedState:
    # first-order/top-level/whatever data:
    # x, y <- GPS
    # velocity <- velocity, control
    # acceleration <- IMU
    # angle <- control, IMU
    # angle_vel <- IMU
    # angle_accel <- IMU

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

    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.acceleration = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_acceleration = 0
    
    #TODO get better fusing algo because this just averages everything
    def update(self, *args):
        total = len(args) # however many states you want is what you get

        for state in args:
            self.x += state.x
            self.y += state.y
            self.velocity += state.velocity
            self.acceleration += state.acceleration

            self.angle += state.angle
            self.angle_velocity += state.angle_velocity
            self.angle_acceleration += state.angle_acceleration
        self.x /= total
        self.y /= total
        self.velocity /= total
        self.acceleration /= total

        self.angle /= total
        self.angle_velocity /= total
        self.angle_acceleration /= total

class LocHandler:
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

def main():
    global locHandler
    global publisher

    # Initalize our node in ROS
    rospy.init_node("fusion_node")
    print("Fusion node initialized!")

    # Create a Publisher that we can use to publish messages to the /daniel/state topic
    publisher = rospy.Publisher("/daniel/state", State, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    #rospy.wait_for_service("/sim/waypoints")
    #waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()
    print("Waypoints aquired!")

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

    publisher.publish(locHandler.getState())


# so if anything imports our code (documentation tools, linters, etc.) it isn't run automatically and things don't get broken
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass