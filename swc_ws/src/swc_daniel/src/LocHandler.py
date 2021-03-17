
class LocHandler:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.angle = 0

        self.prev_x = 0
        self.prev_y = 0
        self.prev_angle = 0

        self.x_vel = 0
        self.y_vel = 0
        self.angle_vel = 0

        self.imu_ran = False
        self.vel_ran = False
        self.control_ran = False

        self.last_t = 0
    
    def imuCallback(self, data):
        #data.oreintation # quaternion (x, y, z, w)
        #data.angular_velocity # vector 3 (x, y, z)
        #data.linear_velocity # vector 3
        # there's also a covariance for each one that tells you how much to trust it or something?
        
        # delta_x / delta_t = vel
        
        t = data.header.stamp.secs + data.header.stamp.nsecs / 1000000000 # nanosecond is one-billionth of a second
        delta_t = t - self.last_t
        
        try:
            self.x_vel = data.linear_acceleration.x / delta_t
        except ZeroDivisionError:
            pass

        print(self.x_vel)        
        self.last_t = t
        
    def velocityCallback(self, data):
        #data.data
        if not self.vel_ran:
            print(data)
            self.vel_ran = True
    
    def controlCallback(self, data):
        #data.speed
        #data.turn_angle
        if not self.control_ran:
            print(data)
            self.control_ran = True