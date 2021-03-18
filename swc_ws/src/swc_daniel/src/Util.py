#from LocHandler import RobotState
from math import sqrt
import tf


class PurePursuit:
    def __init__(self, points, lookahead_distance):
        self.points = points
        self.lookahead_distance = lookahead_distance
        self.fillPoints()
    
    def fillPoints(self):
        finalPoints = []
        for point in self.points:
            finalPoints.append(point)
            #TODO make this actually fill points so we get smooth curves. Or not. Only if it gets us higher score
        self.points = finalPoints

    def getNextHeading(self, curr_state):
        lastPoint = self.points[-1] # did I ever mention how much I love Python and negative indexing?

        # for every point in the list (requires list of points to be sorted in order of when you want to hit them)
        for index, point in enumerate(self.points):
            if dist(curr_state.x, curr_state.y, point[0], point[1]) > self.lookahead_distance: # if we've gone to far
                targetPoint = self.points[index - 1] # then we want the point just before this

                # angle between current and goal (probably/hopefully/blame Justin Z)
                return atan((point[0] - curr_state.x) / (point[1] - curr_state.y))

            elif dist(curr_state.x, curr_state.y, lastPoint[0], lastPoint[1]) < self.lookahead_distance: # if we're pretty much at the end
                return atan((lastPoint[0] - curr_state.x) / (lastPoint[1] - curr_state.y)) # same thing but with the last point
        # if we somehow got here,
        print("What the heck are you doing here?")
        return curr_state.angle

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

"""
un-optimized version
def xyToLatLon(x, y):
    lat0Pos = 35.205853
    lon0Pos = -97.442325

    latitude = (y + noise) / 110944.33 + lat0Pos
    longitude = (x + noise) / 91058.93 + lon0Pos
    return latitude, longitude"""

# this is copied from the sim code
def xyToLatLon(x, y):
    return (y / 110944.33 + 35.205853, x / 91058.93 + -97.442325)

# this is just the inverse of the above
def latLonToXY(lat, lon):
    return ((lat - 35.205853) * 110944.33, (lon - -97.442325) * 91058.93)


# Copied directly from whatever worked in Robot.py
def getYaw(quat):
    explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
    return tf.transformations.euler_from_quaternion(explicit_quat)[2] # get a euler, yaw is second angle of it