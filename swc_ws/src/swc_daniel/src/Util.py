#from LocHandler import RobotState
from __future__ import print_function, division
from math import sqrt, atan, degrees, atan2, cos, sin
import time
import tf


def frange(start, end, step):
    """
    Ooh a docstring yes very programmer much skilled
    This is basically range() but it works with decimal values, exactly like you'd expect. Python you let me down by not having this
    """
    running_total = start
    while running_total + step < end:
        yield running_total
        running_total += step
    yield running_total # b/c it's added but not yielded because then while exits
    yield end # final case

class PurePursuit(object):
    def __init__(self, points, lookahead):
        self.lookahead = lookahead
        self.points = self.fillPoints(points)
        self.goal = (0, 0)
    
    def fillPoints(self, points):
        newPoints = []
        for index, point in enumerate(points): # for each point in the given list of points
            newPoints.append(point) # add that point to our new list
            try:
                nextPoint = points[index + 1] # get the next point
            except IndexError: # if that was the last point
                break # out of loop preferrably, not entire function

            angle = atan((point[0] - nextPoint[0]) / (point[1] - nextPoint[1])) # we don't need to wrap anything here in degrees() because everything expects radians and this returns radians so we good
            distance = pointDist(point, nextPoint) # the distance between current and next point (next point in list of points given)
            for length in frange(0.2, distance, 0.2): # for each point we want to add stepping by 0.2
                x = cos(angle) * length  +  point[0] # add that point along the angle plus our current location
                y = sin(angle) * length  +  point[1] # there's probably a better way to do this but whatever
                newPoints.append((x, y)) # add it to the list

        return newPoints
    
    def getNextHeading(self, state):
        # really un-optimized. Best case is O(1) when we're at the end of the line, second best is O(lookahead / 0.2) when we're just starting,
        # worse case is ~O(n / 0.2) when we're close to the end and have to search through all the points
        # average is like O(n/2 / 0.2) when we're in the middle I guess. If I wanted to I could add like a lastGoalPoint and just start search
        # a couple back from that to get like O(lookahead / 0.2) which is pretty good
        for index, point in enumerate(self.points): # for each of the points in our path
            if dist(state.x, state.y, *point) > self.lookahead: # if the point is too far
                self.goal = self.points[index - 1] # then go back one, because we know the points are sorted because we put them there
            elif pointDist(point, self.points[-1]) < self.lookahead: # if we're seeing the end of the line
                self.goal = self.points[-1]
        
        return #TODO make it return heading


    def getGoalPoint(self):
        return self.goal

class PIDController:
    """A very trashy PID Controller that is basically just WPILib's but python"""
    def __init__(self, kP, kI=0, kD=0, threshold=0.1, velocity_threshold=0.1):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        
        self.error = 0
        self.lastError = 0
        self.velocity_error = 0
        self.totalError = 0

        self.setpoint = 0
        self.threshold = threshold
        self.velocity_threshold = velocity_threshold

        self.time = 0
        self.lasttime = 0
    
    def calculate(self, measurement):
        """Gets the output of the PID based on the setpoint and the given measure of the system's state and stuff"""
        self.error = measurement - self.setpoint
        self.velocity_error = (self.error - self.lastError) / (time.time() - self.lasttime) #idk why it's called velocity that's what WPILib says but it makes sense
        self.totalError += self.error

        output = self.error * self.kP + self.totalError * self.kI + self.velocity_error * self.kD
        self.lastError = self.error
        self.lastTime = time.time()
        return output

    def setSetpoint(self, setpoint):
        """Sets the setpoint of the PID controller"""
        self.setpoint = setpoint
    
    def atSetpoint(self):
        """Gets if we've reached the setpoint (and, if using a kD term, if we've been there long enough kinda)"""
        return abs(self.error) < self.threshold and abs(self.velocity_error) < self.velocity_threshold
    
    def setThreshold(self, threshold):
        """Sets the threshold for if we're at the setpoint or not"""
        self.threshold = threshold

# Good ol' distance formula
def dist(x1, y1, x2, y2):
    """Basically just distance formula"""
    return sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def pointDist(point1, point2):
    """Basically just distance formula but for working with tuples representing points for convienence"""
    return sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

#un-optimized version
#def xyToLatLon(x, y):
#    lat0Pos = 35.205853
#    lon0Pos = -97.442325
#
#    latitude = (y + noise) / 110944.33 + lat0Pos
#    longitude = (x + noise) / 91058.93 + lon0Pos
#    return latitude, longitude

# this is copied from the sim code
def xyToLatLon(x, y):
    """So from the simulator code this is how they do x, y -> lat, lon"""
    return (y / 110944.33 + 35.205853, x / 91058.93 + -97.442325)

# this is just the inverse of the above
def latLonToXY(lat, lon):
    """The inverse of xyToLatLon, which _should_ be accurate as it was copypasta-ed from the simulator code
    +37 because sim considers (0, 0) to be center of field, but we have origin at robot pose"""
    return ((lon - -97.442325) * 91058.93, 37 + (lat - 35.205853) * 110944.33)


# Copied directly from whatever worked in Robot.py
def getYaw(quat):
    """Gets the rotation around the Z axis (so, yaw/heading) from a quaternion. Copied from Ye Olde Robot.py"""
    explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
    return tf.transformations.euler_from_quaternion(explicit_quat)[2] # get a euler, yaw is second angle of it