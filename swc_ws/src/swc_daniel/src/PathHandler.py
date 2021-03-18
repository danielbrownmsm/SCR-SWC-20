from Util import PIDController, PurePursuit, dist

class PathHandler:
    def __init__(self, points):
        self.points = points
        self.path = self.getPath()
        self.ppc = PurePursuit(self.path, 10) #TODO tune lookahead distance
        self.angle_pid = PIDController(0.5, 0.001, 0.000001) #TODO tune
        self.angle_pid.setSetpoint()
        self.vel_pid = PIDController(3, 0.01, 0.0001) #TODO tune
        self.vel_pid.setSetpoint(0) # we want to reduce distance to 0

        # state var
        self.state = None
    
    def getPath(self):
        #TOD0 make this run a pathfinding algo to determine which bonus waypoints to go for b/c if they are too far
        # then they're not worth going for but honestly I don't think we'll run into that problem
        #UPDATE: yeah we're not going to do this. It's not worth it, just hit up all the points
        tempList = []

        for point in self.points:
            y, x = latLonToXY(point.latitude, point.longitude)
            tempList.append((x, y)) # tuple
        
        return tempList
    
    def recalculatePath(self):
        #TODO for when obstacles are in path or something
        pass

    def obstacleCallback(self, data):
        #TODO make this better or something and actually do something etc
        self.obstacles = data.obstacles
        for obstacle in self.obstacles:
            if obstacle in self.path: # this is the main problem
                recalculatePath()

    def stateCallback(self, data):
        self.state = data

    def getTargetHeading(self):
        self.angle_pid.setSetpoint(self.ppc.getNextHeading(self.state)) # let PPC figure out heading
        return self.angle_pid.calculate(self.state.angle) # then we will PID to that
    
    def getTargetVelocity(self):
        return self.vel_pid.calculate(dist(self.state.x, self.state.y, self.goalPoint.x, self.goalPoint.y)) # for velocity slow down when approach
    
    def getMessage(self):
        # THIS SHOULD NOT BE USED FOR CONTROL
        # that should be handled by the ControlHandler. However, the CH needs to get data from somewhere, so here we are
        message = Control()
        message.speed = getTargetVelocity()
        message.turn_angle = getTargetHeading()
        
        return message