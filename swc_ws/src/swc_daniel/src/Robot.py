from swc_msgs.msg import Control
import math
import tf

class Robot():
    def __init__(self, waypoints):
        self.curr_lat = waypoints[0].latitude # have us start out at first location
        self.curr_lon = waypoints[0].longitude # to prevent driving backwards really fast at sim start
        
        self.target_waypoints = waypoints # get waypoint data
        self.waypoint_threshold = 0.00001 # how close we have to be to count a waypoint as reached
        self.lat_error = 0.0 # diff between current and goal latitude
        self.lon_error = 0.0 # same here but for longitude
        self.atBoundaryLat = "none" # GPS boudaries
        self.atBoundaryLon = "none"# top/bottom and right/left
        self.target_index = 1 # start with bonus waypoint 1 as 1st target
        self.changeTarget(self.target_index) # to set first goal
        
        self.curr_angle = 0.0 # initialize, amiright?
        self.x_accel = 0.0
        self.x_velocity = 0.0 # for (maybe) obstacle detection
        
        self.speedP = 175000 # gains for the P controllers
        self.angleP = 10 # angle is a bit whack, speed just go fast lol
        
        self.cam_data = [] # this I understand (not)
        self.num_per_rows = 0 # ooh, docs!
        self.color_data = []

        self.laser_data = [] # list for LIDAR
        self.stripped_data = [] # data after we're done with it (evil grin)
        self.num_laser = 0 # length of stripped data
        self.obstructed_threshold = 35 # threshold for needing to turn
        self.reverse_threshold = 70 # threshold for backing up
        self.obstructed = False
        self.reverse_now = False
    
    # recieve camera stream (not currently working)
    #def updateCamera(self, cam):
    #    self.cam_data = cam.data
    #    # print(cam.data)
    #    self.num_per_rows = cam.steps

    def updateBumper(self, msg):
        if msg.data:
            self.reverse_now = True
        else:
            self.reverse_now = False

    # get and handle LIDAR data
    def updateLaser(self, data):
        self.laser_data = data.ranges # range from robot to object
        self.stripped_data = self.laser_data[215:-20] # we only care about the front-facing part
        self.stripped_data = [x for x in self.stripped_data if x > 0.0001 and x <= 1.25] # strip unwanted numbers
        # too far away (>1.25) and not-detected (0)
        self.num_laser = len(self.stripped_data)
        # if we're going to hit an obstacle
        if self.num_laser >= self.obstructed_threshold:
            self.obstructed = True
        else:
            self.obstructed = False
        
        # if we're running against an obstacle
        if self.num_laser >= self.reverse_threshold:
            self.reverse_now = True

    # updates the robot's current position
    def updateCoords(self, gps):
        self.curr_lat = gps.latitude
        self.curr_lon = gps.longitude
        self.updateTarget() # see if we've achieved goal and then go to the next one
        self.checkBoundaries() # make sure we're not about to fall off
    
    #  and handle goal waypoint stuff
    def updateTarget(self):
        if self.targetReached() and self.target_index == 1: # if we're past 1st waypoint
            self.target_index += 1 # go to next waypoint
            self.changeTarget(self.target_index)
        elif self.targetReached() and self.target_index == 2:
            self.target_index += 1 # go for next
            self.changeTarget(self.target_index)
        elif self.targetReached() and self.target_index == 3:
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
        elif self.curr_lat < 35.20541594:
            self.atBoundaryLat = "bottom"
        else:
            self.atBoundaryLat = "none"
        
        # if we're too far right or left
        if self.curr_lon < -97.4425903447:
            self.atBoundaryLon = "right"
        elif self.curr_lon > -97.4420318775:
            self.atBoundaryLon = "left"
        else:
            self.atBoundaryLon = "none"

    # updates the target we P to
    def changeTarget(self, index):
        self.goal_lat = self.target_waypoints[index].latitude
        self.goal_lon = self.target_waypoints[index].longitude
    
    # implementation of distance formula, for internal use
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) # distance formula, derived from pythagorean theorum
        # Haversine formula would be better, yes, but then XDist would have to be in meters and I don't wanna

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

        # accel/velocity - commented out for now
        # self.x_accel = data.linear_acceleration.x # now for accel
        # self.x_velocity = self.x_velocity + self.x_accel * 0.04
    
    # -- Control --

    # final function call for speed
    def getDesiredSpeed(self):
        if self.reverse_now: # if we're up against an obstacle
            return -8 # back up (untested)
        elif self.obstructed: # if we have an obstacle
            return 4 # go a little slower

        return self.getDist() * self.speedP # Just go fast lol
        
    # final function call for angle
    def getDesiredAngle(self):
        if self.reverse_now:
            if self.curr_angle - self.getNeededAngle() > 0:
                return 30
            #else:
                #return -30 # robot can get stuck in infinite loop
            return 30
        # check LIDAR
        if self.obstructed:
            if self.curr_angle - self.getNeededAngle() > 0:
                return 30
            else:
                return -30
        
        # check boundaries (after LIDAR so it overrides it)
        if self.atBoundaryLon == "left":
            return 30
        elif self.atBoundaryLon == "right":
            return -30
        if self.atBoundaryLat == "top" or self.atBoundaryLat == "bottom":
            return 20 # works barely
        
        return (self.curr_angle - self.getNeededAngle()) * self.angleP # return the error times gain
        
    # function that actually gives the control() message
    def getAction(self):
        control_msg = Control()
        control_msg.speed = self.getDesiredSpeed()
        control_msg.turn_angle = self.getDesiredAngle()
        return control_msg
    
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #