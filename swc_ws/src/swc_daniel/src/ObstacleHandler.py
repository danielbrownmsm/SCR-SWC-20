from __future__ import print_function
from itertools import groupby
import time

class ObstacleHandler:
    def __init__(self):
        self.obstacles = []
        self.laserData = []
        
        self.x = 0
        self.y = 0
    
    def laserCallback(self, data):
        self.laserData = data.ranges

    def stateCallback(self, data):
        self.x = data.x
        self.y = data.y
    
    def visionCallback(self, data):
        pass
    
    def getMessage(self):
        pass