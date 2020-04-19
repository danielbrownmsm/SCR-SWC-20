import math
import tf
from geopy import distance

class PositionHandler:
    def __init__(self, waypoints):
        # stuff we need to publish
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        # GPS stuff
        self.waypoints = waypoints.waypoints
        self.curr_lat = self.waypoints[0].latitude # starting lat
        self.curr_lon = self.waypoints[0].longitude # and starting lon
        
        # Boundaries
        self.atBoundaryLat = "none" # GPS boundaries top/bottom
        self.atBoundaryLon = "none"# and right/left
        # coordinates we need to _not_ go past otherwise fall into the void
        self.boundaries = {"top":35.2063480829, "bottom":35.20541594, "right":-97.4425903447, "left":-97.4420318775}
        
        # Bonus waypoints
        self.waypoint_threshold = 0.00001 # how close we have to be to count a waypoint as reached
        self.target_index = 1 # start with bonus waypoint 1 as 1st target
        self.changeTarget(self.target_index) # to set first goal
    

    # updates the robot's current position
    def updateCoords(self, data):
        self.curr_lat = data.latitude # set current lat
        self.curr_lon = data.longitude # and current lon
        self.updateTarget() # see if we've achieved goal and then go to the next one
        self.checkBoundaries() # make sure we're not about to fall off
    
    def interpretWaypoints(self):
        # TODO make interpret waypoints
        # have it take waypoints and convert to x,y relative to robot starting coordinates
        waypoint1 = (lat1, lon1)
        waypoint2 = (lat2, lon2)
        waypoint3 = (lat2, lon1)
        


    # updates target if we've reached it, else continue
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
    
    # checks to see if we've reached our target or not
    def targetReached(self):
        lat_error = math.fabs(self.curr_lat) - math.fabs(self.goal_lat) # absolute value because we're working with negative
        lon_error = math.fabs(self.curr_lon) - math.fabs(self.goal_lon) # coordinates and stuff
        return math.fabs(lat_error) < self.waypoint_threshold and math.fabs(lon_error) < self.waypoint_threshold and self.getDist() < self.waypoint_threshold

    # checks to see if we've passed our boundaries and updates variables accordingly for later functions to use
    def checkBoundaries(self):
    # if we're too far up or down
    if self.curr_lat > self.boundaries["top"]: self.atBoundaryLat = "top"
    elif self.curr_lat < self.boundaries["bottom"]: self.atBoundaryLat = "bottom"
    else: self.atBoundaryLat = "none"
    
    # if we're too far right or left
    if self.curr_lon < self.boundaries["right"]: self.atBoundaryLon = "right"
    elif self.curr_lon > self.boundaries["left"]: self.atBoundaryLon = "left"
    else: self.atBoundaryLon = "none"

    