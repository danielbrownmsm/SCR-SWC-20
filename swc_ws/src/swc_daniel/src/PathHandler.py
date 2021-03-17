class PathHandler:
    def __init__(self, points):
        self.points = points
        self.path = self.getPath()
    
    def getPath(self):
        #TODO make this run a pathfinding algo to determine which bonus waypoints to go for b/c if they are too far
        # then they're not worth going for but honestly I don't think we'll run into that problem
        return self.points