class Robot:
    def __init_(self, kP, kD, threshold, velocityThreshold):
        self.kP = kP
        self.kD = kD
        self.threshold = threshold
        self.velocityThreshold = velocityThreshold

    def test(self, generation):
        os.system("roslaunch swc_daniel swc_daniel.launch & echo TODOSIMULATOR")
        #TODO write self vals to vals.txt
        sleep(30) # wait what?
        with open("/path/to/sim/results.txt", "r") as f:
            try:
                self.fitness = float(f.readlines()[6].strip()[6:]) # the 6th line skipping the first 6 characters of "Score: "
            except Exception: # if something happened (ie the simulator didn't finish)
                self.fitness = -1 # get outa' here
        #TODO write results to file to keep track of progress and, you know, see which is best
        #.csv: HEADER,GENERATION,KP,KD,THRESHOLD,VELOCITY_THRESHOLD,FITNESS

    def breed(self, other): #TODO get better breeding than just averaging
        kP = (self.kP + other.kP) / 2
        kD = (self.kD + other.kD) / 2
        threshold = (self.threshold + other.threshold) / 2
        velocityThreshold = (self.velocityThreshold + other.velocityThreshold) / 2
        
        return Robot(kP, kD, threshold, velocityThreshold)

    def mutate(self): #TODO this seems like VERY BAD HACK code so please fix
        #random.choice(self.kP, self.kD, self.threshold, self.velocityThreshold) += random.randint(-1, 1) * 0.9 # it gets increased or decreased by 10%
        which = random.randint(0, 4)
        if which == 0:
            self.kP = randomChange(self.kP)
        elif which == 1:
            self.kD = randomChange(self.kD)
        elif which == 2:
            self.threshold = randomChange(self.threshold)
        else:
            self.velocityThreshold = randomChange(self.velocityThreshold)

# returns a value increased or decreased by 10%
def randomChange(val):
    return val + (val * random.randint(-1, 1) * 0.1)

robots = []
#TODO create initial robots

breed_pool = []
next_generation = []
for generation in range(50): # 50 generations
    for robot in robots:
        robot.test(generation)
        
        for chance in range(int(1 / robot.fitness  *  1000)):
            breed_pool.append(robot)
    
    for new_bot in range(10): # ten robots / gen ???
        next_generation.append(random.choice(breed_pool).breed(random.choice(breed_pool))) # breed each with another
    
    robots = next_generation