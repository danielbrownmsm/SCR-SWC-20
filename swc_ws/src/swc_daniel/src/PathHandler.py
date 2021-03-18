from Util import PIDController, PurePursuit, dist

class PathHandler:
    def __init__(self, points):
        self.points = points
        self.path = self.getPath()
        self.ppc = PurePursuit(self.path)
        self.vel_pid = PIDController(3, 0.01, 0.0001) #TODO tune
        self.vel_pid.setSetpoint(0) # we want to reduce distance to 0

        # state vars
        self.x = 0
        self.y = 0
        self.velocity = 0
        self.angle = 0
        self.angle_vel = 0
    
    def getPath(self):
        #TODO make this run a pathfinding algo to determine which bonus waypoints to go for b/c if they are too far
        # then they're not worth going for but honestly I don't think we'll run into that problem
        # then do a PPC fill points or something?
        return self.points
    
    def recalculatePath(self):
        #TODO for when obstacles are in path or something
        pass

    def obstacleCallback(self, data):
        #TODO make this better or something and actually do something etc
        self.obstacles = data.obstacles
        for obstacle in self.obstacles:
            if obstacle in self.path
                recalculatePath()

    def stateCallback(self, data):
        self.x = data.x
        self.y = data.y
        self.velocity = data.velocity

        self.angle = data.angle
        self.angle_velocity = data.angle_velocity

    def getTargetHeading(self):
        return self.ppc.getNextHeading(self.x, self.y, self.angle)
    
    def getTargetVelocity(self):
        return self.vel_pid.calculate(dist(self.x, self.y, self.goalPoint.x, self.goalPoint.y))
    
    def getMessage(self):
        message = Control()
        message.speed = 0  # even though max speed is 8 there is random noise so we should go with something above it so we always go as fast as possible
        message.turn_angle = 0
        
        return message