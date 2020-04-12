from swc_msgs.msg import Control
import math
# import numpy as np
import tf

class Robot():
    def __init__(self, waypoints):
        self.curr_lat = waypoints[0].latitude # have us start out at first location
        self.curr_lon = waypoints[0].longitude # to prevent driving backwards really fast at sim start
        
        self.target_waypoints = waypoints # this is a comment
        self.aboutToFallOff = False # if we miss the final waypoint and are going to fall off edge
        self.target_index = 1 # start with bonus waypoint 1 as 1st target
        self.changeTarget(self.target_index) # to set first goal
        
        self.curr_angle = 0.0 # initialize, amiright?
        self.x_accel = 0.0
        self.velocity = 0.0
        
        self.speedP = 150000 # gains for the P controllers
        self.angleP = 5.3 # angle is a bit whack, speed just go fast lol
        
        self.cam_data = [] # this I understand (not)
        self.num_per_rows = 0 # ooh, docs!
        self.color_data = []

        self.laser_data = [] # list for LIDAR
        self.stripped_data = [] # data after we're done with it (evil grin)
        self.num_laser = 0
        self.obstructed_threshold = 40
        self.reverse_threshold = 80
        self.obstructed = False
        self.reverse_now = False
    
    # recieve camera stream
    def updateCamera(self, cam):
        self.cam_data = cam.data
        # print(cam.data)
        self.num_per_rows = cam.steps

    # get and handle LIDAR data
    def updateLaser(self, data):
        self.laser_data = data.ranges # range from robot to object
        self.stripped_data = self.laser_data[215:-20] # we only care about the front-facing part
        #print(self.stripped_data)
        self.stripped_data = [x for x in self.stripped_data if x > 0.00001 and x <= 1.5] # strip all 0's and really low #'s (thx SO)
        # as well as anything too far away (>1.5)
        self.num_laser = len(self.stripped_data)
        #print(self.num_laser)
        # if we're going to hit an obstacle
        if self.num_laser >= self.obstructed_threshold:
            self.obstructed = True
        else:
            self.obstructed = False
        
        # if we're running against an obstacle
        if self.num_laser >= self.reverse_threshold:
            self.reverse_now = True
        else:
            self.reverse_now = False


    # updates the robot's current position
    def updateCoords(self, gps):
        self.curr_lat = gps.latitude
        self.curr_lon = gps.longitude
        self.updateTarget()
    
    #  and handle goal waypoint stuff
    def updateTarget(self):
        if self.curr_lat > self.target_waypoints[1].latitude and self.target_index == 1: # if we've made it past 1st waypoint
            self.target_index += 1 # go to next waypoint
            self.changeTarget(self.target_index)
        elif self.curr_lat > self.target_waypoints[2].latitude and self.target_index == 2:
            self.target_index += 1 # go for next
            self.changeTarget(self.target_index)
        elif self.curr_lat > self.target_waypoints[3].latitude and self.target_index == 3:
            self.target_index += 1
            self.changeTarget(self.target_index)
        
        if self.curr_lat > self.target_waypoints[4].latitude:
            self.aboutToFallOff = True
        else:
            self.aboutToFallOff = False

    # updates the target we P to
    def changeTarget(self, index):
        self.goal_lat = self.target_waypoints[index].latitude
        self.goal_lon = self.target_waypoints[index].longitude
    
    # implementation of distance formula, for internal use
    def dist(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2) # distance formula, derived from pythagorean theorum
    
    # returns distance between robot and goal
    def getDist(self):
        return self.dist(self.curr_lon, self.curr_lat, self.goal_lon, self.goal_lat)
    
    # needed for the angle that we want to drive at
    def getXDist(self):
        return self.goal_lon - self.curr_lon # idk if this is right, might need to flip
    
    # gives us the angle that we want to get to
    def getNeededAngle(self):
        return math.asin(self.getXDist() / self.getDist()) # trig. this is _soh_cahtoa. So sin(angle) gives opposite / hypot. So asin(opposit / hypot) gives angle (asin is inverse sine)
    
    # updates our current angle from IMU data
    def updateIMU(self, data):
        quat = data.orientation # we're interested in orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
        euler = tf.transformations.euler_from_quaternion(explicit_quat) # get a euler
        self.curr_angle = euler[2] # and update the yaw

        self.x_accel = data.linear_acceleration.x # now for accel
        self.velocity = self.velocity + self.x_accel * 0.04
        #print(self.velocity)
    
    # -- Control --

    # final function call for speed
    def getDesiredSpeed(self):
        if self.reverse_now: # if we're up against an obstacle
            return -8
        return self.getDist() * self.speedP # it's WORKING! but honestly you don't need a PID for this. Just go fast lol
        
    # final function call for angle
    def getDesiredAngle(self):
        if self.aboutToFallOff:
            return -20
        if self.obstructed:
            if self.curr_angle - self.getNeededAngle() > 0:
                return 30
            else:
                return 30
        return (self.curr_angle - self.getNeededAngle()) * self.angleP # It's WORKING! but anglePID is messed up. And we understeer a lot
        # that's why we have a x5
        
    # function that actually gives the control() message
    def getAction(self):
        control_msg = Control()
        control_msg.speed = self.getDesiredSpeed()
        control_msg.turn_angle = self.getDesiredAngle()
        return control_msg
    
    # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # # #