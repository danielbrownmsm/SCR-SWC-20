#from LocHandler import RobotState
from math import sqrt

#TODO DOCUMENT *EVERYTHING* LIKE *EVERYTHING EVERYTHING*

class PurePursuit:
    def __init__(self, points):
        self.points = points
        self.fillPoints()
    
    def fillPoints(self):
        #TODO fill points
        pass

    def getNextHeading(self, curr_state):
        #TODO get next heading
        pass

class PIDController:
    def __init__(self, kP, kI=0, kD=0, threshold=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        
        self.error = 0
        self.lastError = 0
        self.totalError = 0

        self.setpoint = 0
        self.threshold = 0

        self.time = 0
        self.lasttime = 0
    
    def calculate(measurement, setpoint):
        self.setpoint = setpoint
        self.calculate(measurement)
    
    def calculate(measurement):
        self.error = measurement - setpoint
        self.totalError += self.error

        output = self.error * self.kP + self.totalError * self.kI + (self.error - self.lastError) / (time.time() - self.lasttime) * self.kD
        self.lastError = error
        self.lastTime = time.time()
        return output

    def setSetpoint(self):
        self.setpoint = setpoint
    
    def atSetpoint(self):
        return abs(self.error) < self.threshold
    
    def setThreshold(self, threshold):
        self.threshold = threshold

#TODO write Kalman Filter or whatever
class KalmanFilter:
    def __init__(self):
        pass

    def update(self, val):
        pass

    def get(self):
        return 0

# Good ol' distance formula
def dist(x1, y1, x2, y2):
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

#TODO haversine
def haversine(lat1, lon1, lat2, lon2):
    return 0

# Copied directly from whatever worked in Robot.py
def getYaw(quat):
    explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
    return tf.transformations.euler_from_quaternion(explicit_quat)[2] # get a euler, yaw is second angle of it