import random

# a class to represent a robot, for genetic evolution of robots
class GeneticRobot():
	def __init__(self, minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold):
		self.minSpeed = minSpeed
		self.maxSpeed = maxSpeed
		self.speedP = speedP

		self.minAngle = minAngle
		self.maxAngle = maxAngle
		self.angleP = angleP

		self.timerCallback = timerCallback
		self.waypoint_threshold = waypoint_threshold
		
		self.time_ = 0
	
	def setTime(self, x):
		self.time_ = x
	
	def __lt__(self, other): # magic methods go brrr
		return self.time_ < other.time_ # we sort by time, lowest wins

with open("values.txt", 'r') as vf:
	values = vf.read()

robot = GeneticRobot(values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7])

with open("results.txt", 'r') as rf:
	result = rf.read()

robot.setTime(result)