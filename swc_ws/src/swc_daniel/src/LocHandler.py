import time
from Util import getYaw, latLonToXY
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
        self.i_vel = 0

        self.i_time = 0
        self.i_last_time = 0
        self.i_first_run = True

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

        print("Localization handler initialized!")
    
    # this data is mostly trash
    def imuCallback(self, data):
        self.i_angle_vel = data.angular_velocity.z
        self.i_angle = degrees(getYaw(data.orientation))

        """if not self.i_first_run:
            self.i_time = data.header.stamp.secs# + data.header.stamp.nsecs
            delta_time = self.i_time - self.i_last_time
            self.i_last_time = self.i_time
        
            self.i_vel += data.linear_acceleration.x * delta_time
        else: # wait a run so we can get a time in there so something or IDK honestly at this point my brain is melting and <random nonsense />
            self.i_time = data.header.stamp.secs# * 1000000 + data.header.stamp.nsecs
            self.i_last_time = self.i_time
            self.i_first_run = False"""

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
        self.g_y, self.g_x = latLonToXY(data.latitude, data.longitude)
        self.g_y += 37 # robot starts at (-37, 0) b/c I guess (0, 0) is in far left corner

    def getState(self):
        state = State()
        #TODO figure out something to do with event
        #TODO filter everything
        
        #TODO do fusion with angle
        self.angle = self.i_angle
        
        self.velocity = self.v_vel #TODO change, filter, weight
        
        #TODO velocity is broken b/c collisions so ??? add in bumper?
        #self.velocity = self.i_vel
        #TODO IMU velocity is crap plz fix

        #print(self.velocity)

        #SOH CAH TOA
        self.x += self.velocity * cos(radians(self.angle)) # wait what don't I need to divide by time or something WHAT?
        self.y += self.velocity * sin(radians(self.angle)) # according to Justin's code I need to multiply? I mean it works pretty well as-is

        #TODO weight and make it where GPS doesn't just overwhelm well no because that's good and when would we be stopped anyways
        self.x = (self.x + self.g_x) / 2
        self.y = (self.y + self.g_y) / 2

        #x += delta x / delta time
        
        state.x = self.x
        state.y = self.y
        state.velocity = self.velocity
        
        state.angle = self.angle
        state.angle_velocity = self.angle_velocity

        # print position (drifts like ~5 meters with no noise on after driving wack across the whole field)
        print(str(self.x) + ", " + str(self.y))
        
        return state