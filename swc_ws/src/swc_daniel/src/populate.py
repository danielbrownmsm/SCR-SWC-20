from genetic import GeneticRobot
import random

print("Populating...")
for x in range(0, 50): # 100 should be good for now, at 300s per robot
#self, minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold
    if x == 25:
        print("Halfway there...")
    minSpeed = random.randint(0, 4)
    maxSpeed = random.randint(6, 8)
    speedP = random.randint(100000, 200000)
    minAngle = random.randint(0, 3)
    maxAngle = random.randint(30, 45)
    angleP = random.randint(5, 20)
    timerCallback = 0.1 # just do this for now. can tweak later
    waypoint_threshold = 0.00001
    with open("/mnt/c/Users/Brown_Family01/Documents/GitHub/SCR-SWC-20/swc_ws/src/swc_daniel/src/all_values.txt", "a") as f:
        x = GeneticRobot(minSpeed, maxSpeed, speedP, minAngle, maxAngle, angleP, timerCallback, waypoint_threshold)
        y = x.getValues()
        f.write(str(y) + "\n")
print("Done!")