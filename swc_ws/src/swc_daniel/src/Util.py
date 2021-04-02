#from LocHandler import RobotState
from __future__ import print_function, division
from math import sqrt, atan, degrees, atan2, cos, sin
import time
import tf


def float_range(start, end, step):
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

class PurePursuit:
    """A very trashy pure-pursuit controller"""
    def __init__(self, points, lookahead_distance):
        self.points = points
        self.goalPoint = [0, 0]
        self.lookahead_distance = lookahead_distance
        self.fillPoints()
    
    def fillPoints(self):
        """Fills the points between goals so we can do good turning and stuff probably"""
        finalPoints = []
        someRandomThreshold = 0.2 # yeah every this meters seems fine
        
        for index, point in enumerate(self.points): # for each point in our list of goals
            print("Adding point: " + str(point[0]) + ", " + str(point[1]))
            finalPoints.append(point) # add the point
            # atan2 b/c preserving sign or something that we might want idk
            angle = degrees(atan2(point[1], point[0])) # that's probably right order of params TODO check
            
            # for every point in between current and next increasing by someRandomThreshold along the same angle
            try:
                for hyp in float_range(0, dist(point[0], point[1], self.points[index+1][0], self.points[index+1][1]), someRandomThreshold):
                    new_x = degrees(cos(angle)) * hyp # wait I don't need to recalc the cos() and sin() vals they stay the same
                    new_y = degrees(sin(angle)) * hyp # but "premature optimization is the root of all evil" - some programmer so I'll just TODO make faster
                    finalPoints.append((new_x, new_y)) # add that point
            except IndexError as e:
                print(e)
                print("In Util.py in PurePursuit in fillPoints on " + str(point[0]) + ", " + str(point[1]))
                
        self.points = finalPoints
        print("Points filled!")

    def getNextHeading(self, curr_state):
        """Gets the next heading we should take based on our current location and the path"""
        #TODO rewrite this plz this looks terrible and is most likely wrong
        lastPoint = self.points[-1] # did I ever mention how much I love Python and negative indexing?

        # for every point in the list (requires list of points to be sorted in order of when you want to hit them)
        for index, point in enumerate(self.points):
            if dist(curr_state.x, curr_state.y, point[0], point[1]) > self.lookahead_distance: # if we've gone to far
                targetPoint = self.points[index - 1] # then we want the point just before this
                self.goalPoint = targetPoint

                # angle between current and goal (probably/hopefully/blame Justin Z)
                return degrees(atan((targetPoint[0] - curr_state.x) / (targetPoint[1] - curr_state.y)))

            elif dist(curr_state.x, curr_state.y, lastPoint[0], lastPoint[1]) < self.lookahead_distance: # if we're pretty much at the end
                self.goalPoint = lastPoint
                return degrees(atan((lastPoint[0] - curr_state.x) / (lastPoint[1] - curr_state.y))) # same thing but with the last point
        # if we somehow got here,TODO i think there's an edge case I'm missing
        print("What the heck are you doing here?")
        return curr_state.angle
    
    def getGoalPoint(self):
        """Gets the goal point we are returning for heading so our distance PID can use it for distance PID purposes"""
        # !todo eliminate the reason for this being here oh wait it's here for distance PID purposes nvm
        return self.goalPoint

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