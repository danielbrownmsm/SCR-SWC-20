import random, os, re

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

		self.missed_waypoint = False		
		self.time_ = 0
	
	def setTime(self, new_time):
		self.time_ = new_time
	
	def __lt__(self, other): # magic methods go brrr
		return self.time_ < other.time_ # we sort by time, lowest wins
	
	def mutate(self):
		x = random.randint(0, 100):
		# 1% chance of mutation
		if x == 1:
			x = random.randint(0, 7)
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
			elif x == 6:
				self.timerCallback += random.uniform(-0.02, 0.02)
				if self.timerCallback <= 0.01:
					self.timerCallback = 0.02
			elif x == 7:
				# standard is 0.00001
				self.waypoint_threshold += random.uniform(-0.000005, 0.000005)
	
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
			vf.write(self.getValues)
		os.system("cd ~")
		os.system("cd Documents/GitHub/SCR-SWC-20/src")
		os.system("roslaunch swc_daniel swc_daniel.launch")
		# and then launch simulator as well
		time.sleep(300) # wait for roslaunch to run and sim to close
		with open("results.txt") as rf:
			try:
				self.score = float(re.search(r"(?<=Score: )\d+\.*\d*", rf.read()).group()) # matches 0.1, 1., and 1 if preceded by Score: 
			except Exception:
				print("Error occured:")
				print(Exception)
				self.score = 10000000

	def getValues(self):
		return [self.minSpeed, self.maxSpeed, self.speedP, self.minAngle, self.maxAngle, self.angleP, self.timerCallback, self.waypoint_threshold]
		

class Controller():

	self.generation = 0

	def __init__(self):
		self.population = []
		self.breed_pool = []
	
	def populate(self):
		for x in range(0, 100): # 100 should be good for now, at 300s per robot
		#self, minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold
			minSpeed = random.randint(0, 4)
			maxSpeed = random.randint(6, 8)
			speedP = random.randint(100000, 200000)
			minAngle = random.randint(1, 10)
			maxAngle = random.randint(30, 45)
			angleP = random.randint(5, 20)
			timerCallback = random.uniform(0.01, 0.1)
			waypoint_threshold = 0.00001
			self.population[x] = GeneticRobot(minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold)
	
	def loop(self):
		print("Testing...")
		for robot in self.population:
			robot.fitness()
		
		print("Assembling...")
		for robot in self.population:
			n = robot.score
			for x in range(0, n):
				self.breed_pool.append(self.robot)
		
		print("Breeding...")
		for robot in self.breed_pool: # for every robot
			a = random.randint(0, len(self.breed_pool))
			b = random.randint(0, len(self.breed_pool))
			self.breed_pool[a].breed(self.breed_pool[b]) # breed robot at loc a with robot at loc b
		
		self.population.sort()
		print("Generation: " + str(self.generation))
		print("Best Score: " + str(self.population[0].score))
		print("Worst Score: " + str(self.population[-1].score))

		with open("best.txt", "a") as bf: # append values
			bf.write(self.population[0].getValues())
		self.generation += 1

controller = Controller()
controller.populate()
while True:
	controller.loop()