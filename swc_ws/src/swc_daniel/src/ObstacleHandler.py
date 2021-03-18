from __future__ import print_function
from itertools import groupby
from swc_msgs.msg import Obstacle, Obstacles

class ObstacleHandler:
    def __init__(self):
        self.obstacles = []
        self.laserData = []
        
        self.x = 0
        self.y = 0

        print("Obstacle handler initialized!")
    
    def laserCallback(self, data):
        self.laserData = data.ranges

    def stateCallback(self, data):
        self.x = data.x
        self.y = data.y
    
    def visionCallback(self, data):
        pass
    
    def getMessage(self):
        # this is so freaking hard to read like what's an obs and an Obstacle and a elcatsbo and all that
        # but whatever eye dhont kcair

        msg_list = []
        for obstacle in self.obstacles:
            obs = Obstacle()
            obs.x = obstacle.x
            obs.y = obstacle.y
            obs.width = obstacle.width
            msg_list.append(obs)
        msg = Obstacles()
        msg.obstacles = msg_list

        return msg

class Obstacle_:
    def __init__(self, x, y, width):
        self.x = x
        self.y = y
        self.width = width
    
    def collidesWidth(self, x1, y1, x2, y2):
        return False