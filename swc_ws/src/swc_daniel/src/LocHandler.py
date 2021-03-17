import time

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
        #TODO init vars or something
        pass
    
    # this data is mostly trash
    def imuCallback(self, data):
        self.i_angle_vel = data.angular_velocity.z
        self.i_vel = 0 #TODO fix
        self.i_angle = data.oreintation

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
        #data.latitude
        #data.longitude
        self.g_x = 0
        self.g_y = 0
        #TODO haversine formula
        #TODO convert this to meters or something on a grid and stuff

    def getPos():
        #TODO do fusion
        #TODO integrate for pos
        #TODO filter everything
        return RobotState(self.x, self.y, self.angle, self.vel, self.angle_vel)