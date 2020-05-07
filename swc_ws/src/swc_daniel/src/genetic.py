import random

# a class to represent a robot, for genetic evolution of robots
class Robot():
	def __init__(self, minSpeed, speedP, angleP, time_):
		self.minSpeed = minSpeed
		self.speedP = speedP
		self.angleP = angleP
		self.time_ = time_
	
	def __lt__(self, other): # magic methods go brrr
		return self.time_ < other.time_ # we sort by time, lowest wins
		

# handles evolving all our robots
class Controller():
	def __init__(self, robots):
		self.robots = robots
		self.new_robots = []
	
	# changes some values by a little bit
	def mutate(self):
		for robot in self.robots:
			x = random.randint(0, 100)
			if x % 2 == 0: # if the number is even
				continue # skip it. this yields 50% chance of mutation. Probably too high
			robot.minSpeed += random.random() # this number could be decimal optimized
			robot.speedP += random.randint(-50, 50) # we need this number to be big, so change it by a lot
			robot.angleP += random.random() # same for this
		return
	
	def contest(self):
		self.robots.sort() # calls the __lt__ thing we made
		# THIS MIGHT SORT IN REVERSE SO WE ARE DELETING TOP 25% (BECAUSE LOWEST SCORE SHOULD WIN, BUT I THINK
		# WE'RE DOING HIGHEST SCORE WINS HERE. SO DOUBLE CHECK)
		self.robots = self.robots[:-int(len(self.robots) * 0.25)] # *should* remove worst 25%
	
	def breed(self):
		for i in range(0, len(self.robots)): # for however many robots we have
			minSpeed = self.robots[random.randint(0, len(self.robots))].minSpeed # assign new values to them from existing robots
			speedP = self.robots[random.randint(0, len(self.robots))].speedP # this represents them breeding, to a degree
			angleP = self.robots[random.randint(0, len(self.robots))].angleP
			# yes, this allows for some robots to have multiple children or to have none at all. 
			# Boo hoo. If they're good they'll live long enough to have children
			self.new_robots[i] = [minSpeed, speedP, angleP]
		return

# run program from bash script, this happens:
with open('robots.txt', 'r') as f:
	values = f.read()
#values.handle_the_data_by_converting_into_list_of_GeneRobot_instances(NaN)
controller = Controller(values)

controller.mutate()
controller.contest()
controller.breed()

# the bash script writes... AH SCREW IT DINNER
with open('robots.txt', 'w') as ff:
	ff.write(controller.new_robots)
# and then it'll loop in the script so we good here.