from swc_msgs.msg import Control
import math
import tf

class Robot():
    def __init__(self, waypoints, values):
        # order is minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold
        self.time = 0.0
        
        self.curr_lat = waypoints[0].latitude # have us start out at first location
        self.curr_lon = waypoints[0].longitude # to prevent driving backwards really fast at sim start
        
        self.waypoint_threshold = values[7] # how close we have to be to count a waypoint as reached
        self.lat_error = 0.0 # diff between current and goal latitude
        self.lon_error = 0.0 # same here but for longitude
        
        self.target_waypoints = waypoints # get waypoint data
        self.target_index = 1 # start with bonus waypoint 1 as 1st target
        self.changeTarget(self.target_index) # to set first goal
        
        self.atBoundaryLat = "none" # GPS boudaries
        self.atBoundaryLon = "none" # top/bottom and right/left
        
        self.curr_angle = 0.0 # initialize, amiright?

        self.speedP = values[2] # gains for the P controllers
        self.angleP = values[5] # angle is a bit whack, speed just go fast lol

        self.min_angle = values[3]
        self.max_angle = values[4]

        self.min_speed = values[0]
        self.max_speed = values[1]

    # hacky timer stuff for logging purposes
    def updateTime(self):
        self.time += 0.01
        
    # updates the robot's current position
    def updateCoords(self, gps):
        self.curr_lat = gps.latitude
        self.curr_lon = gps.longitude
        self.updateTarget() # see if we've achieved goal and then go to the next one
        self.checkBoundaries() # make sure we're not about to fall off
    
    #  and handle goal waypoint stuff
    def updateTarget(self):
        if self.targetReached() and self.target_index == 1: # if we're past 1st waypoint
            print("[" + str(self.time) + "] Waypoint 1 reached!")
            self.target_index += 1 # go to next waypoint
            self.changeTarget(self.target_index)
        elif self.targetReached() and self.target_index == 2:
            print("[" + str(self.time) + "] Waypoint 2 reached!")
            self.target_index += 1 # go for next
            self.changeTarget(self.target_index)
        elif self.targetReached() and self.target_index == 3:
            print("[" + str(self.time) + "] Waypoint 3 reached!")
            self.target_index += 1
            self.changeTarget(self.target_index)
    
    def targetReached(self):
        self.lat_error = math.fabs(self.curr_lat) - math.fabs(self.goal_lat)
        self.lon_error = math.fabs(self.curr_lon) - math.fabs(self.goal_lon)
        return math.fabs(self.lat_error) < self.waypoint_threshold and math.fabs(self.lon_error) < self.waypoint_threshold and self.getDist() < self.waypoint_threshold
    
    def checkBoundaries(self):
        # if we're too far up or down
        if self.curr_lat > 35.2063480829:
            self.atBoundaryLat = "top"
            print("[" + str(self.time) + "] At boundary top!")
        elif self.curr_lat < 35.20541594:
            self.atBoundaryLat = "bottom"
            print("[" + str(self.time) + "] At boundary bottom!")
        else:
            self.atBoundaryLat = "none"
        
        # if we're too far right or left
        if self.curr_lon < -97.4425903447:
            self.atBoundaryLon = "right"
            print("[" + str(self.time) + "] At boundary right!")
        elif self.curr_lon > -97.4420318775:
            self.atBoundaryLon = "left"
            print("[" + str(self.time) + "] At boundary left!")
        else:
            self.atBoundaryLon = "none"

    # updates the target we P to
    def changeTarget(self, index):
        self.goal_lat = self.target_waypoints[index].latitude
        self.goal_lon = self.target_waypoints[index].longitude
        # print("[" + str(self.time) + "] Updating target: " + str(index))
    
    # implementation of distance formula, for internal use
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) # distance formula, derived from pythagorean theorum
        # Haversine formula would be better, yes, but then XDist would have to be in meters and I don't wanna convert

    # returns distance between robot and goal
    def getDist(self):
        return self.dist(self.curr_lon, self.curr_lat, self.goal_lon, self.goal_lat)
    
    # needed for the angle that we want to drive at
    def getXDist(self):
        return self.goal_lon - self.curr_lon # this works no touchy
    
    # gives us the angle that we want to get to
    def getNeededAngle(self):
        return math.asin(self.getXDist() / self.getDist()) # trig. this is _soh_cahtoa. So sin(angle) gives
        # opposite / hypot. So asin(opposit / hypot) gives angle (asin is inverse sine)
    
    # updates our current angle from IMU data
    def updateIMU(self, data):
        quat = data.orientation # we're interested in orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
        euler = tf.transformations.euler_from_quaternion(explicit_quat) # get a euler
        self.curr_angle = euler[2] # and update the yaw

    # -- Control --

    # final function call for speed
    def getDesiredSpeed(self):
        if (self.getDist() * self.speedP) < self.min_speed:
            print("[" + str(self.time) + "] ROBOT GO WHIR")
            return self.min_speed
        elif (self.getDist() * self.speedP) > self.max_speed:
            return self.max_speed # return max speed. doesn't really do anything but it might
        return self.getDist() * self.speedP # Just go fast lol
        
    # final function call for angle
    def getDesiredAngle(self):
        # check boundaries
        if self.atBoundaryLon == "left":
            return 20
        elif self.atBoundaryLon == "right":
            return -20
        if self.atBoundaryLat == "top" or self.atBoundaryLat == "bottom":
            return 20 # works barely
        
        val = (self.curr_angle - self.getNeededAngle()) * self.angleP # return the error times gain
        #if val > self.max_angle:
        #    print("[" + str(self.time) + "] Too much angle")
        #    return self.max_angle
        #elif val < self.min_angle:
        #    print("[" + str(self.time) + "] Not enough angle")
        #    return self.min_angle
        return val
        
    # function that actually gives the control() message
    def getAction(self):
        control_msg = Control()
        control_msg.speed = self.getDesiredSpeed()
        control_msg.turn_angle = self.getDesiredAngle()
        return control_msg
    
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #