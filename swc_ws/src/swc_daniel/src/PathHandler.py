from Util import PIDController, PurePursuit, dist, latLonToXY
from swc_msgs.msg import Control, State

class PathHandler:
    def __init__(self, points):
        self.points = points
        self.path = self.getPath()
        self.ppc = PurePursuit(self.path, 10) #TODO tune lookahead distance
        self.angle_pid = PIDController(0.5, 0.001, 0.000001) #TODO tune
        self.vel_pid = PIDController(3, 0.01, 0.0001) #TODO tune
        self.vel_pid.setSetpoint(0) # we want to reduce distance to 0

        # state var
        self.state = State()
        self.state.x = 0
        self.state.y = 0
        self.state.angle = 0
        print("Path handler initialized!")

        self.ticks = 0
    
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
                self.recalculatePath()

    def stateCallback(self, data):
        self.state = data
        self.ticks += 1

        #if self.ticks % 10 == 0:
        #    print(data)

    def getTargetHeading(self):
        self.angle_pid.setSetpoint(self.ppc.getNextHeading(self.state)) # let PPC figure out heading
        #print("tracking heading: " + str(self.angle_pid.setpoint))
        return self.angle_pid.calculate(self.state.angle) # then we will PID to that
    
    def getTargetVelocity(self):
        self.goalPoint = self.ppc.getGoalPoint()
        #if self.ticks % 10 == 0:
        #    print("Goal point: (" + str(self.goalPoint[0]) + ", " + str(self.goalPoint[1]) + ")")
        return self.vel_pid.calculate(dist(self.state.x, self.state.y, self.goalPoint[0], self.goalPoint[1])) # for velocity slow down when approach
    
    def getMessage(self):
        # THIS SHOULD NOT BE USED FOR CONTROL
        # that should be handled by the ControlHandler. However, the CH needs to get data from somewhere, so here we are
        message = Control()
        message.speed = self.getTargetVelocity()
        message.turn_angle = self.getTargetHeading()
        
        return message