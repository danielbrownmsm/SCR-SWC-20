from LocHandler import RobotState

class PurePursuit:
    def __init__(self, points):
        self.points = points
        self.fillPoints()
    
    def fillPoints(self):
        #TODO
        pass

    def getNextHeading(self, curr_state):
        #TODO
        pass

class PIDController:
    def __init__(self, kP, kI=0, kD=0, threshold=0):
        self.kP = kP
        self.kI = kI
        self.kD = kD
        
        self.error = 0
        self.lastError = 0

        self.setpoint = 0
        self.threshold = 0 #TODO
    
    def calculate(measurement, setpoint=self.setpoint):
        #TODO write actual stuff
        self.error = measurement - setpoint

        #TODO fix
        return self.kP * self.error + self.kI * self.error + self.kD * self.error

    def setSetpoint(self):
        self.setpoint = setpoint
    
    def atSetpoint(self):
        return abs(self.error) < self.threshold
    
    def setThreshold(self, threshold):
        self.threshold = threshold
    
    def setPID(self, P=self.kP, I=self.kI, D=self.kD):
        self.kP = P
        self.kI = I
        self.kD = D

#TODO distance formula
def dist():
    pass

#TODO haversine
def haversine():
    pass

#TODO test this, doc this
def getYaw(quat):
    explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
    return tf.transformations.euler_from_quaternion(explicit_quat)[2] # get a euler, yaw is second angle of it