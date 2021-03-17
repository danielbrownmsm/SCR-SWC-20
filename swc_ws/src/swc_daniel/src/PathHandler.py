class PathHandler:
    def __init__(self, points):
        self.points = points
        self.path = self.getPath()
    
    def getPath(self):
        #TODO make this run a pathfinding algo to determine which bonus waypoints to go for b/c if they are too far
        # then they're not worth going for but honestly I don't think we'll run into that problem
        return self.points
    
    def recalculatePath(self):
        #TODO for when obstacles are in path or something
        pass

    def obstacleCallback(self, data):
        #TODO
        pass

    def stateCallback(self, data):
        #TODO
        pass

    def getTargetHeading(self):
        #TODO
        pass
    
    def getTargetVelocity(self):
        #TODO
        pass
    
    def getMessage(self):
        #TODO
        pass