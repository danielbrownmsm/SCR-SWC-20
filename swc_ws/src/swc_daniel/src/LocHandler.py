import time
from Util import getYaw, haversine
from math import degrees, radians, sin, cos
from swc_msgs.msg import State

class RobotState:
    def __init__(self, x, y, angle, velocity, angle_velocity):
        self.x = x
        self.y = y
        self.angle = angle
        self.velocity = velocity
        self.angle_velocity = angle_velocity
        self.time = time.time()

class LocHandler:
    def __init__(self, startCoords):
        # "official" (or "final" or whatever) vars
        self.x = 0
        self.y = 0
        self.angle = 0

        self.prev_x = 0
        self.prev_y = 0
        self.prev_angle = 0
        self.angle_velocity = 0

        self.time = 0
        self.lasttime = 0

        # IMU vars
        self.i_angle = 0
        self.i_angle_vel = 0
        self.i_angle = 0

        # velocity vars
        self.v_vel = 0

        # control vars
        self.c_vel = 0
        self.c_angle = 0
        self.c_angle_change = 0

        # GPS vars
        self.g_x = 0
        self.g_y = 0
        self.startLat = startCoords.latitude
        self.startLon = startCoords.longitude
    
    # this data is mostly trash
    def imuCallback(self, data):
        self.i_angle_vel = data.angular_velocity.z
        self.i_vel = 0 #TODO fix
        self.i_angle = degrees(getYaw(data.orientation))

    # this data is *chef's kiss* perfecto
    def velocityCallback(self, data):
        # max speed is 8 m/s
        self.v_vel = data.data
    
    # this data is ok
    def controlCallback(self, data):
        self.c_vel = data.speed
        self.c_angle_change = data.turn_angle

    # this data is kinda ok
    def gpsCallback(self, data):
        self.g_x, self.g_y = haversine(self.startLat, self.startLon, data.latitude, data.longitude)
        #TODO convert this to meters or something on a grid and stuff

    def getState(self):
        state = State()
        #TODO figure out something to do with event
        #TODO filter everything
        
        #TODO do fusion with angle
        self.angle = self.i_angle
        
        #TODO add in IMU accelerometer data
        self.velocity = (self.v_vel + self.c_vel * 3) / 4 #TODO change, filter, weight
        #TODO velocity is broken b/c collisions so ??? add in bumper?

        #SOH CAH TOA
        self.x += self.velocity * cos(radians(self.angle))
        self.y += self.velocity * sin(radians(self.angle))

        #x += delta x / delta time
        
        state.x = self.x
        state.y = self.y #TODO fix
        state.velocity = self.velocity
        
        state.angle = self.angle
        state.angle_velocity = self.angle_velocity

        # print position (drifts like ~5 meters with no noise on after driving wack across the whole field)
        #print(str(self.x) + ", " + str(self.y))

        return state