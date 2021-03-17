import time
from Util import getYaw, haversine

class RobotState:
    def __init__(self, x, y, angle, velocity, angle_velocity):
        self.x = x
        self.y = y
        self.angle = angle
        self.velocity = velocity
        self.angle_velocity = angle_velocity
        self.time = time.time()


class LocHandler:
    def __init__(self):
        # "official" (or "final" or whatever) vars
        self.x = 0
        self.y = 0
        self.angle = 0

        self.prev_x = 0
        self.prev_y = 0
        self.prev_angle = 0

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

        # GPS vars
        self.g_x = 0
        self.g_y = 0
    
    # this data is mostly trash
    def imuCallback(self, data):
        self.i_angle_vel = data.angular_velocity.z
        self.i_vel = 0 #TODO fix
        print(data.orientation)
        #self.i_angle = getYaw(data.oreintation)

    # this data is *chef's kiss* perfecto
    def velocityCallback(self, data):
        # max speed is 8 m/s
        self.v_vel = data.data
    
    # this data is ok
    def controlCallback(self, data):
        self.c_vel = data.speed
        self.c_angle = data.turn_angle

    # this data is kinda ok
    def gpsCallback(self, data):
        self.g_x, self.g_y = haversine(self.startLat, self.startLon, data.latitude, data.longitude)
        #TODO convert this to meters or something on a grid and stuff

    def getPos():
        #TODO do fusion
        #TODO integrate for pos
        #TODO filter everything
        return RobotState(self.x, self.y, self.angle, self.vel, self.angle_vel)