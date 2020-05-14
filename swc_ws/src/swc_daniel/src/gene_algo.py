import random
import os
import re

# thanks, internet
def merge(dict1, dict2):
    result = {**dict1, **dict2}
    return result

class DNA():
    def __init__(self, key, value, rateOfChange):
        self.key = key
        self.value = value
        self.rateOfChange = rateOfChange
    
    def getRateOfChange(self):
        return self.rateOfChange
    
    def getKey(self):
        return self.key
    
    def getValue(self):
        return self.value

# a class to represent a robot, for genetic evolution of robots
class GeneticRobot():
	def __init__(self, dna, mutationRate=1, sortLowestFirst=False): # DNA should be a list of DNA objects
        self.dna = dna
        self.sortLowestFirst = sortLowestFirst
        self.mutationRate = mutationRate # expressed as percentage, *NOT* decimal

		self.fitness = 0
	
	def setScore(self, new_score):
		self.score = new_score
	
	def __lt__(self, other): # magic methods go brrr
        if self.sortLowestFirst:
    		return self.score < other.score # we sort by time, lowest wins
        return self.score > other.score
	
	def mutate(self):
		x = random.randint(0, 100) # because rate is a percentage
		if x < mutationRate:
			x = random.randint(0, len(self.dna))
			self.dna[x].value += random.uniform(-self.dna[x].getRateOfChange(), self.dna[x].getRateOfChange())
	
	def breed(self, partner):
		x = random.randint(0, 1)
        partnerDNA = partner.dna
		if x == 1:
            otherDNA = patnerDNA[:round(len(partnerDNA) / 2)] # split in half this way
            selfDNA = self.dna[round(len(partnerDNA / 2)):]
            newDNA = merge(selfDNA, otherDNA)
			new_bot = GeneticRobot(newDNA)
		else:
            otherDNA = patnerDNA[round(len(partnerDNA) / 2):] # split in half the other way (lit just changed colon pos)
            selfDNA = self.dna[:round(len(partnerDNA / 2))]
            newDNA = merge(selfDNA, otherDNA)
			new_bot = GeneticRobot(newDNA)
        
        new_bot.mutate() # stuff happens, okay?!? Sometimes robots gotta change. And that's cool. Everyone's fine.
		return new_bot
	
	def getValues(self):
		return self.dna
	
    def fitness(self):
		with open("values.txt", "w") as vf:
			x = self.getValues()
			vf.write(str(x))
		#os.system("cd ~")
		#os.system("cd Documents/GitHub/SCR-SWC-20/src")
		os.system("/mnt/c/Users/Brown_Family01/Documents/SCR_SWC_20_SIM_5.0_WIN/SCRSWC20.exe & roslaunch swc_daniel swc_daniel.launch")
		# and then launch simulator as well
		#time.sleep(60) # wait for roslaunch to run and sim to close
		# minute per robot * 100 robots * 10(?) generations seems reasonable enough
		#os.system("rosnode kill --all")
		with open("results.txt", "r") as rf:
			try:
				self.score = float(re.search(r"(?<=Score: )\d+\.*\d*", rf.read()).group()) # matches 0.1, 1., and 1 if preceded by Score: 
				print("Score: " + str(self.score))
			except Exception:
				print("Did Not Finish (or REGEX is messed up)")
				#print(Exception)
				self.score = 10000000

class Controller():
	def __init__(self, bestFile, allValuesFile, resultsFile):
		self.bestFile = bestFile
		self.allValuesFile = allValuesFile
		self.resultsFile = resultsFile

		self.population = []
	
	def doPopulate(self):
		#TODO

	def doFitness(self):
		#TODO
	
	def doAssemble(self):
		#TODO

	def doBreed(self):
		#TODO