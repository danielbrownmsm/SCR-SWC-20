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

# for LIDAR you do stuff
# for camera you do really complicated things that I don't feel like doing right now

# first-order/top-level/whatever data:
# x, y <- GPS
# velocity <- velocity, control
# acceleration <- IMU
# angle <- control, IMU
# angle_vel <- IMU
# angle_accel <- IMU

# pretty sure sensor data becomes noisier the faster you go
# latitude stdv = 1.843
# longitude stdv = 2.138
# accel stdv = 0.15
# orientation stdv = 0.017
# angle vel stdv = 0.017
# vel stdv = 0.1
# angle stdv = 0.4
# power stdv = 0.1
# LIDAR and camera don't have noise (I know camera doesn't, pretty sure LIDAR doesn't)

# while !started:
#   disregard sensor data (we know we're not moving, we know where we start)

# =======================================================================================================================
# TODO WAIT NO MAKE AN INIT_STATE METHOD OF ALL THE HANDLERS THAT IS CALLED AT THE BEGINNING SO THEY KNOW THEIR POSITIONS (starting waypoints!) AND CAN GET THE PREV_STATE VAR INITED AND STUFF DANIEL YOU'RE A GENIUS (occasionaly)
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
        self.accel = 0

        self.angle = 0

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data):
        if self.hasRun:
            self.time = time.time()
            delta_time = self.time - self.prev_state.time
            
            #TODO find a way to pass angle data here because without it we're pretty much useless here as velocity and we need to share our velocity and what the heck it's like late right now why are these lines so long?
            self.angle = None
        
            self.velocity = data.data # who wants proper terminology anyways?
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            self.x = self.prev_state.x + (self.velocity) * delta_time #TODO do trig here to get sep x/y components
            self.y = self.prev_state.y + (self.velocity) * delta_time
            
            self.prev_state = State(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else:
            # wait if we know this is the first then wouldn't the start be (-37, 0) or whatever?
            self.x, self.y = (0, 0) #TODO fix init-ing states
            self.prev_state = State(self.x, self.y, 0, 0, 0, 0, 0, time.time(), 0, DEFAULT_TRUST)
            self.hasRun = True

class ControlHandler:
    # pretty much the same as the IMU but Ackermann so blah and also terrible because it's terrible like what if you are running into an obstacle?
    def __init__(self):
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.accel = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_accel = 0

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data):
        if self.hasRun:
            self.time = time.time()
            delta_time = self.time - self.prev_state.time
            
            # angle before position because integrated for position needs the angle for the trig
            #TODO fix because we only actual change our heading if we are going forwards so 20 deg at 1 m/s for X seconds is less than 20 deg at 4 m/s
            self.angle = self.prev_state.angle + data.turn_angle
            self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time

            self.velocity = data.speed # who wants proper terminology anyways?
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            self.x = self.prev_state.x + (self.velocity) * delta_time #TODO do trig here to get sep x/y components
            self.y = self.prev_state.y + (self.velocity) * delta_time
            
            self.prev_state = State(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else:
            # wait if we know this is the first then wouldn't the start be (-37, 0) or whatever?
            self.x, self.y = (0, 0) #TODO fix init-ing states
            self.prev_state = State(self.x, self.y, 0, 0, 0, 0, 0, time.time(), 0, DEFAULT_TRUST)
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
        self.accel = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_accel = 0

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data):
        if self.hasRun:
            self.time = time.time()
            delta_time = self.time - self.prev_state.time
            
            # angle before position because integrated for position needs the angle for the trig
            self.angle = getYaw(data.orientation) #TODO fuse angle with integrated angular velocity and vice versa
            self.angle_velocity = data.angular_velocity.z
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time

            self.acceleration = data.linear_acceleration.x # probably x, right?
            self.velocity = self.prev_state.velocity + self.acceleration * delta_time # yeah that's right
            self.x = self.prev_state.x + (self.velocity) * delta_time #TODO do trig here to get sep x/y components
            self.y = self.prev_state.y + (self.velocity) * delta_time
            
            self.prev_state = State(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else:
            # wait if we know this is the first then wouldn't the start be (-37, 0) or whatever?
            self.x, self.y = (0, 0) #TODO fix init-ing states
            self.prev_state = State(self.x, self.y, 0, 0, 0, 0, 0, time.time(), 0, DEFAULT_TRUST)
            self.hasRun = True

class GpsHandler:
    #delta_time = time - last_time
    #x, y = latLonToXY(gps.lat, gps.lon)
    #velocity = dist(x, y, last_x, last_y) / delta_time
    #acceleration = (velocity - last_velocity) / delta_time
    #angle = atan((y - last_y) / (x - last_x))
    #angle_velocity = (angle - last_angle) / delta_time
    #angle_acceleration = (angle_velocity - last_angle_velocity) / delta_time
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
        self.accel = 0

        self.angle = 0
        self.angle_velocity = 0
        self.angle_accel = 0

        self.time = 0
        self.hasRun = False
        self.prev_state = None
    
    def update(self, data):
        if self.hasRun: # if we've already had a state to compare to
            self.time = time.time()
            delta_time = self.time - self.prev_state.time

            self.x, self.y = latLonToXY(data.latitude, data.longitude)
            self.velocity = dist(self.x, self.y, self.prev_state.x, self.prev_state.y) / delta_time
            self.acceleration = (self.velocity - self.prev_state.velocity) / delta_time
            
            self.angle = atan((self.y - self.prev_state.y) / (self.x - self.prev_state.x))
            self.angle_velocity = (self.angle - self.prev_state.angle) / delta_time
            self.angle_acceleration = (self.angle_velocity - self.prev_state.angle_velocity) / delta_time
    
            self.prev_state = State(self.x, self.y, self.velocity, self.acceleration, self.angle, self.angle_velocity, self.angle_acceleration, self.time, self.prev_state.time, DEFAULT_TRUST)
        else: # otherwise, make a state assuming we haven't moved and at our current position but wait
            # we know where we start it's the first waypoint also (-37, 0) so what do we do?
            self.x, self.y = latLonToXY(data.latitude, data.longitude)
            self.prev_state = State(self.x, self.y, 0, 0, 0, 0, 0, time.time(), 0, DEFAULT_TRUST)
            self.hasRun = True
        

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