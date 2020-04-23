from swc_msgs import Position
from math import cos, sin, fabs, sqrt, radians, asin

class LocalizationHandler:
    def __init__(self, converted_waypoints):
        self.heading = 0.0
        self.x = 0.0
        self.y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        self.dt = 0.1 # ==TEMPTEMP VAL==

        self.v_velocity = 0.0 # data from velocity publisher
        self.c_velocity = 0.0 # from Control
        self.a_velocity = 0.0 # from acceleration (IMU)
        self.vel = 0.0

        self.acceleration = 0.0 # getting from callback
        self.prev_acceleration = 0.0

        self.converted_waypoints = converted_waypoints
        self.target_index = 1
        self.changeTarget(self.target_index)
    
    #
    # ===waypoint achievement management=== (call updateTarget() somewhere
    # else nothing happens)

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
        self.x_error = fabs(self.x) - fabs(self.goal_x)
        self.y_error = fabs(self.y) - fabs(self.goal_y)
        return fabs(self.x_error) < self.margin and fabs(self.y_error) < self.margin

    # updates the target we P to
    def changeTarget(self, index):
        self.goal_x = self.converted_waypoints[index][1] # order is (lat, lon) which translates
        self.goal_y = self.converted_waypoints[index][0] # to (y, x)

    #
    # ===velocity managment===
    #

    # callback for velocity
    def velocityCallback(self, data):
        self.v_velocity = data.data
    
    # callback for control_msg
    def controlCallback(self, data):
        self.c_velocity = data.speed
    
    # callback for IMU
    def imuCallback(self, data):
        quat = data.orientation # we're interested in orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
        euler = tf.transformations.euler_from_quaternion(explicit_quat) # get a euler
        self.heading = euler[2] # and update the heading

        self.acceleration = data.linear_acceleration.x
        self.velocity = self.prev_acceleration + self.acceleration * 0.04 # "imu refreshes at ~25Hz" so 0.04
        self.prev_acceleration = self.acceleration # so, you know, we actually update

    def getVelocity(self):
        return ((self.c_velocity * 2) + (self.v_velocity * 4) + self.a_velocity) / 7 # average velocity, with less weight
        # given to velocity got from acceleration and more given to velocity publisher
    
    def updatePosition(self):
        self.vel = self.getVelocity()
        self.x = self.x + self.vel * cos(self.heading) * self.dt
        self.y = self.y + self.vel * sin(self.heading) * self.dt
    
    # ===publishing===
    # and publish!
    def getMessage(self):
        positionMessage = Position()
        positionMessage.x = self.x
        positionMessage.y = self.y
        positionMessage.heading = self.heading
        return positionMessage