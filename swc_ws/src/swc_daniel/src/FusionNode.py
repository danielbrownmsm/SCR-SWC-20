DEFAULT_TRUST = {
    "x":1,
    "y":1,
    "velocity":1,
    "accel":1,
    "angle":1,
    "angle_velocity":1,
    "angle_accel":1
}

class State:
    def __init__(self, x=0, y=0, velocity=0, accel=0, angle=0, angle_velocity=0, angle_accel=0, time=0, prev_time=0, trust_vals=DEFAULT_TRUST):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.accel = accel

        self.angle = angle
        self.angle_velocity = angle_velocity
        self.angle_accel = angle_accel

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

class GpsState:
    def __init__(self, x=0, y=0, velocity=0, accel=0, angle=0, angle_velocity=0, angle_accel=0, time=0, prev_time=0, trust_vals=DEFAULT_TRUST):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.accel = accel

        self.angle = angle
        self.angle_velocity = angle_velocity
        self.angle_accel = angle_accel

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
    
    def solvePosition(self):
        pass
    
    # position/linear
    def solvePosition(self):
        return
    
    def solveVelocity(self):
        pass
    
    def solveAccel(self):
        pass
    
    # angle
    def solveAngle(self):
        pass
    
    def solveAngleVel(self):
        pass
    
    def solveAngleAccel(self):
        pass

# for the gps you do
#delta_time = time - last_time
# then
#x, y = latLonToXY(gps.lat, gps.lon)
# then
#velocity = dist(x, y, last_x, last_y) / delta_time
# then
#acceleration = (velocity - last_velocity) / delta_time
# then TOA
#angle = atan((y - last_y) / (x - last_x))
# then
#angle_velocity = (angle - last_angle) / delta_time
# then
#angle_acceleration = (angle_velocity - last_angle_velocity) / delta_time
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


class LocHandler:
    def __init__(self):
        self.imu_state = State()
        self.gps_state = State()
        self.vel_state = State()
        self.ctrl_state = State()
        self.lidar_state = State()
        self.vision_state = State()
    
        self.fused_state = State()

    def gpsCallback(self, data):
        pass
    
    def imuCallback(self, data):
        pass

    def controlCallback(self, data):
        pass

    def velocityCallback(self, data):
        pass

    def lidarCallback(self, data):
        pass

    def visionCallback(self, data):
        pass
    
    def getState(self, data):
        return -1