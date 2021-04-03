import matplotlib.pyplot as plt
import random
import time
from Util import PIDController, clamp, sign

controller = PIDController(1, 0.0001, 0.0005, velocity_threshold=0.5, threshold=0.5)
controller.setSetpoint(100)
random.seed(314159)

pos = 0
vel = 0
friction = 0.1

positions = []
times = []
outputs = []
vels = []
iteration = 0

controller.calculate(0)

while iteration < 1000:
    input_ = pos + (random.normalvariate(0, 2)*vel) # add noise to input
    
    output = clamp(controller.calculate(input_), -7, 7) # get output and clamp it
    output += random.normalvariate(0, 0.1) # randomise output so dead-reckoning doesn't work

    vel += output / 17 # v += F/m
    vel += vel * friction * -sign(vel) # adds friction opposing velocity
    vel = clamp(vel, -20, 20) # limit our velocity (simulating max velocity on something)

    pos += vel # update position based on velocity

    # add all our data
    times.append(iteration)
    positions.append(pos)
    outputs.append(output)
    vels.append(vel)

    iteration += 1 # step one up
    time.sleep(0.00002) # sleep so our time.times() work

setpoint_list = [controller.setpoint for i in range(len(times))] # to plot a straight line at setpoint

fig, ax = plt.subplots()
plt.plot(positions, color="blue", label="position")
plt.plot(outputs, color="red", label="output")
#plt.plot(vels, color="yellow", label="velocity")
plt.plot(setpoint_list, color="green", label="setpoint")
plt.legend()
plt.show()