import random, os

# hard seed: 636929

class Robot(object):
    def __init__(self, kP, kD, threshold, velocityThreshold):
        self.kP = kP
        self.kD = kD
        self.threshold = threshold
        self.velocityThreshold = velocityThreshold

        self.fitness = 0

    def test(self, generation):
        with open("vals.txt", "w") as f: # write our specific parameters to the file to be used by ControlNode
            strings = [str(robot.kP), str(robot.kD), str(robot.threshold), str(robot.velocityThreshold)]
            strings = [string+"\n" for string in strings]
            f.writelines(strings)

        # launch testing programs
        os.system("""~/Desktop/"Daniel's Stuff"/tools/SCR_SWC_20_SIM_6.0_WIN/SCR_SWC_20_SIM_6.0_WIN/SCRSWC20.exe & roslaunch swc_daniel swc_daniel.launch""")

        # get results
        with open("""/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/results.txt""", "r") as f:
            try:
                self.fitness = float(f.readlines()[6].strip()[6:]) # the 6th line skipping the first 6 characters of "Score: "
            except Exception: # if something happened (ie the simulator didn't finish)
                self.fitness = -1 # get outa' here

        #TODO kill all running processes after done. Actually no we'll just do it like last time and have control node kill all

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

# returns a value increased or decreased by a random percent
def randomChange(val):
    return val + (val * random.randint(-1, 1) * round(random.random()/5, 2))

POP_SIZE = 5
MAX_GENERATIONS = 50

robots = []
for i in range(POP_SIZE): # initialize the starting pool
    kP = randomChange(3)
    kD = randomChange(0.6)
    threshold = randomChange(2)
    velocityThreshold = randomChange(6)
    robots.append(Robot(kP, kD, threshold, velocityThreshold))

breed_pool = []
next_generation = []
for generation in range(MAX_GENERATIONS): # 50 generations

    # make sure each generation has it's own seed
    seed = random.randint(0, 1000000)
    with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/simulator.cfg", "r+") as f:
        text = f.readlines() # get text
        text[9] = "Seed=" + str(seed) + "\n" # replace seed
        
        f.seek(0)
        f.writelines(text) # overwrite seed

    # for each robot
    for robot in robots:
        robot.test(generation) # test it

        with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/all_results.txt", "a") as f: # record results
            string = ", ".join([str(generation), str(robot.kP), str(robot.kD), str(robot.threshold), str(robot.velocityThreshold), str(robot.fitness)])
            f.write(string + "\n")
        
        for chance in range(int(1 / robot.fitness  *  1000)): # add it to the breed pool proportional to it's score
            breed_pool.append(robot)
    
    for new_bot in range(POP_SIZE): # ten robots / gen ???
        next_generation.append(random.choice(breed_pool).breed(random.choice(breed_pool))) # breed each with another
    
    random.choice(next_generation).mutate() # mutate one of them
    
    robots = next_generation