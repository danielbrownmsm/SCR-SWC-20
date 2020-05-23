import random
import os
import re

def convert(x):
	#x = x.strip()
	#x = x.replace("'", "").replace("\"", "").replace(" ", "")
	#x = float(x)
	return x

# a class to represent a robot, for genetic evolution of robots
class GeneticRobot():
	def __init__(self, minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold):
		self.minSpeed = convert(minSpeed)
		self.maxSpeed = convert(maxSpeed)
		self.speedP = convert(speedP)

		self.minAngle = convert(minAngle)
		self.maxAngle = convert(maxAngle)
		self.angleP = convert(angleP)

		self.timerCallback = convert(timerCallback)
		self.waypoint_threshold = convert(waypoint_threshold)

		self.score = 0
	
	def setScore(self, new_score):
		self.score = new_score
	
	def __lt__(self, other): # magic methods go brrr
		return self.score < other.score # we sort by time, lowest wins
	
	def mutate(self):
		x = random.randint(0, 100)
		# 1% chance of mutation
		if x == 1:
			x = random.randint(0, 5)
			if x == 0:
				self.minSpeed += random.uniform(-1, 1)
			elif x == 1:
				self.maxSpeed += random.uniform(-1, 1)
			elif x == 2:
				self.speedP += random.randint(-20, 20)
			elif x == 3:
				self.minAngle += random.uniform(-5, 5)
			elif x == 4:
				self.maxAngle += random.uniform(-5, 5)
			elif x == 5:
				self.angleP += random.uniform(-1, 1)
			#elif x == 6:
			#	self.timerCallback += random.uniform(-0.02, 0.02)
			#	if self.timerCallback <= 0.01:
			#		self.timerCallback = 0.02
			#elif x == 7:
			#	# standard is 0.00001
			#	self.waypoint_threshold += random.uniform(-0.000005, 0.000005)
	
	def breed(self, partner):
		x = random.randint(0, 1)
		if x == 1:
			new_bot = GeneticRobot(partner.minSpeed, self.maxSpeed, partner.speedP, self.minAngle, partner.maxAngle, self.angleP, partner.timerCallback, self.waypoint_threshold)
		else:
			new_bot = GeneticRobot(self.minSpeed, partner.maxSpeed, self.speedP, partner.minAngle, self.maxAngle, partner.angleP, self.timerCallback, partner.waypoint_threshold)
		new_bot.mutate()
		return new_bot
	
	def fitness(self):
		with open("values.txt", "w") as vf:
			x = self.getValues()
			vf.write(str(x))
		os.system("/mnt/c/Users/Brown_Family01/Documents/SCR_SWC_20_SIM_5.0_WIN/SCRSWC20.exe & roslaunch swc_daniel swc_daniel.launch")

		with open("results.txt", "r") as rf:
			try:
				self.score = float(re.search(r"(?<=Score: )\d+\.*\d*", rf.read()).group()) # matches 0.1, 1., and 1 if preceded by Score: 
				print("Score: " + str(self.score))
			except Exception:
				print("Did Not Finish (or REGEX is messed up)")
				self.score = 10000000

	def getValues(self):
		return [self.minSpeed, self.maxSpeed, self.speedP, self.minAngle, self.maxAngle, self.angleP, self.timerCallback, self.waypoint_threshold]
		

class Controller():
	def __init__(self):
		self.generation = 0
		self.population = []
		self.breed_pool = []
	
	#def populate(self):
	#	print("Populating...")
	#	for x in range(0, 100): # 100 should be good for now, at 300s per robot
	#	#self, minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold
	#		minSpeed = random.randint(0, 4)
	#		maxSpeed = random.randint(6, 8)
	#		speedP = random.randint(100000, 200000)
	#		minAngle = random.randint(1, 10)
	#		maxAngle = random.randint(30, 45)
	#		angleP = random.randint(5, 20)
	#		timerCallback = round(random.uniform(0.01, 0.1), 4)
	#		waypoint_threshold = 0.00001
	#		self.population[x] = GeneticRobot(minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold)
	
	def loop(self):
		print("Testing...")
		x = 1
		for robot in self.population:
			print("Gen " + str(self.generation) + "Robot " + str(x))
			x += 1
			robot.fitness()
		
		print("Assembling...")
		for robot in self.population:
			n = round((1 / robot.score) * 1000, 0) # smaller score, larger number. Larger score, smaller number
			for x in xrange(0, int(n)): # xrange() is slightly faster and we need speeeeeeeeeeeeed
				self.breed_pool.append(robot)
		
		print("Breeding...")
		x = 0
		for robot in self.population: # for every robot
			a = random.randint(0, len(self.breed_pool)-1)
			b = random.randint(0, len(self.breed_pool)-1)
			robot = self.breed_pool[a].breed(self.breed_pool[b]) # breed robot at loc a with robot at loc b and make the population that
		
		self.population.sort()
		print("Generation: " + str(self.generation))
		print("Best Score: " + str(self.population[0].score))
		print("Worst Score: " + str(self.population[-1].score))

		with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/swc_daniel/src/best.txt", "a") as bf: # append values
			x = str(self.population[0].getValues())
			x += " ==> " + str(self.population[0].score)
			bf.write(str(x))
		
		with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/swc_daniel/src/all_values.txt", "w") as f:
			f.write("")
			# to make sure we have an empty file
		

		with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/swc_daniel/src/all_values.txt", "a") as f:
			all_robots = []
			for robot in self.population:
				f.write(str(robot.getValues()))
				f.write("\n")

		self.generation += 1

print("Initialized...")
controller = Controller()

# read from all_values.txt and put into list of lists of numbers
print("Reading file...")
all_values = []
with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/swc_daniel/src/all_values.txt", "r") as f:
	values = f.readlines() # get list of lines
	for value in values: # iterate over that list
		more_values = value.replace("[", "").replace("]", "").replace("'", "").replace("\"", "").strip().split(",") # to get a list of values
		for string in more_values:
			string = float(string) # convert to floats
		all_values.append(more_values) # put that into all_values

print("Creating list from file...")
for robot in all_values:
	x = GeneticRobot(robot[0], robot[1], robot[2], robot[3], robot[4], robot[5], robot[6], robot[7])
	controller.population.append(x)

print("Entering loop...")
#controller.populate()
while True:
	controller.loop()