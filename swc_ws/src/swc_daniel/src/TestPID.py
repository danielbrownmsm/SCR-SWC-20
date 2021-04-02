import matplotlib.pyplot as plt
import random
import time
from Util import PIDController, clamp, sign

controller = PIDController(0.7, 0.00001, 0.001, velocity_threshold=0.5, threshold=0.5)
controller.setSetpoint(100)

pos = 0
vel = 0
friction = 0.5

errors = []
times = []
iteration = 0

controller.calculate(0)

#while not controller.atSetpoint() and iteration < 700:
while iteration < 300:
    output = controller.calculate(pos + random.normalvariate(0, 1)*vel) # randomizes input (and thus output) proportional to speed
    output = clamp(output, -2, 2)
    #if abs(output) < 0.3 and random.random() > 0.2: # randomly if output is very small
    #    output /= 100 # it will "be ignored" (simulating minimum output required to do things like overcoming friction)
    vel += output
    vel += vel * friction * -sign(vel) # - random.random()/100 # adds randomness to friction (very little)
    vel = clamp(vel, -5, 5)
    pos += vel

    times.append(iteration)
    errors.append(pos)
    iteration += 1
    time.sleep(0.0002)

print(controller.atSetpoint())
print(controller.zero_divisions)
print(controller.velocity_error)

setpoint_list = []
for i in range(len(times)):
    setpoint_list.append(controller.setpoint)

fig, ax = plt.subplots()
plt.plot(times, errors)
plt.plot(times, setpoint_list)
plt.show()