from math import atan, sqrt, fabs, degrees
from swc_msgs.msg import Control

class ControlHandler:
    def __init__(self):
        # no explanation needed for these
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        # or this for that matter
        self.goal_x = 0.0
        self.goal_y = 0.0

        # these comments just break the code into managable chunks
        self.x_dist = 0.0
        self.y_dist = 0.0

        # these have been tested. NOT. NO. TEST PLEASE. MAKE WORK GOOD.
        self.angle_p = 8
        self.speed_p = 175000

        # if we are currently against an obstacle and going to die
        self.reverse = False

        self.time = 0.0
    
    def bumperCallback(self, data):
        if data.data:
            self.reverse = True
            print("[" + str(self.time) + "] Reversing!")
            return
        self.reverse = False

    # callback to get Postion message data
    def update(self, data):
        # oh yeah BTW units are in meters (I hope)
        self.x = data.x
        self.y = data.y
        self.heading = data.heading
        
        self.goal_x = data.goal_x
        self.goal_y = data.goal_y

        self.x_dist = self.goal_x - self.x
        self.y_dist = self.goal_y - self.y

        self.time = data.time
    
    # gives us angle we need to turn to using trig
    def getTurnAngle(self):
        try:
            needed = degrees(atan(self.x_dist / self.y_dist)) # SOHCAHTOA
            error = fabs(needed) - self.heading
        except ZeroDivisionError:
            #print("[" + str(self.time) + "] dcube05 tried to divide by zero")
            return 0
        #print(error)
        return error #* self.angle_p # PIDs are too slow
    
    # gives us the speed we should drive at
    def getSpeed(self):
        if self.reverse: # if we be hurtin
            return -8 # we need to get outta here
        
        # else we good
        val = sqrt(self.x_dist ** 2 + self.y_dist ** 2) * self.speed_p # pythagorean theorem plus a P controller
        if val > 8:
            return 8 # to not possibly overflow anything (that won't happen, riiiiiight?)
        elif val < 2:
            #print("[" + str(self.time) + "] Too slow!")
            return 2 # to not go slow
        return val
        
    
    # returns the message for publishing
    def getMessage(self):
        controlMessage = Control()
        controlMessage.speed = self.getSpeed()
        controlMessage.turn_angle = self.getTurnAngle()
        return controlMessage