import matplotlib.pyplot as plt
import random
import time
from Util import PIDController, clamp, sign, MedianFilter, MovingAverage

controller = PIDController(5, 0, 0.06, velocity_threshold=0.5, threshold=0.5)
controller.setSetpoint(100)
random.seed(314159)

pos = 0
vel = 0
friction = 0.1

positions = []
times = []
inputs = []
outputs = []
filters = []
vels = []
iteration = 0

median = MedianFilter(5)
avg = MovingAverage(1)

controller.calculate(0)

while iteration < 200:
    input_ = pos + (random.normalvariate(0, 1)*vel) # add noise to input
    median.update(input_)
    avg.update(input_)

    filtered = avg.get()
    #filtered = median.get()

    #output = clamp(controller.calculate(input_), -8, 8) # get output and clamp it
    output = clamp(controller.calculate(filtered), -1, 1) # get output and clamp it
    output += random.normalvariate(0, 0.1) # randomise output so dead-reckoning doesn't work

    vel += output / 2 # v += F/m
    vel = clamp(vel, -8, 8) # limit our velocity (simulating max velocity on something)
    vel += vel * friction * -sign(vel) # adds friction opposing velocity

    pos += vel # update position based on velocity

    # add all our data
    times.append(iteration)
    positions.append(pos)
    filters.append(filtered)
    inputs.append(input_)
    #outputs.append(output)
    #vels.append(vel)

    iteration += 1 # step one up
    time.sleep(0.00002) # sleep so our time.times() work

setpoint_list = [controller.setpoint for i in range(len(times))] # to plot a straight line at setpoint

average_error = 0
for position in positions:
    average_error += controller.setpoint - position
average_error /= len(positions)
print(average_error)

fig, ax = plt.subplots()
#plt.plot(outputs, color="red", label="output")
#plt.plot(vels, color="yellow", label="velocity")
plt.plot(setpoint_list, color="green", label="setpoint")
plt.plot(filters, color="red", label="filters")
plt.plot(inputs, color="orange", label="inputs")
plt.plot(positions, color="yellow", label="position")
plt.legend()
plt.show()