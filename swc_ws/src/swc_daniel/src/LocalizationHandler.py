from swc_msgs import Position

class LocalizationHandler:
    def __init__(self, waypoint_data):
        self.heading = 0.0
        self.x = 0.0
        self.y = 0.0

        self.v_velocity = 0.0 # data from velocity publisher
        self.c_velocity = 0.0 # from Control
        self.a_velocity = 0.0 # from acceleration (IMU)

        self.acceleration = 0.0 # getting from callback
        self.prev_acceleration = 0.0

        self.waypoint_data = waypoint_data
        self.start_point = (self.waypoint_data[0].latitude, self.waypoint_data[0].longitude)
    
    #
    # ===velocity managment===
    #

    # callback for velocity
    def velocityCallback(self, data):
        self.v_velocity = data.Float32
    
    # callback for control_msg
    def controlCallback(self, data):
        self.c_velocity = data.speed
    
    # callback for IMU
    def imuCallback(self, data):
        quat = data.orientation # we're interested in orientation
        explicit_quat = [quat.x, quat.y, quat.z, quat.w] # this is a workaround for types not playing nice
        euler = tf.transformations.euler_from_quaternion(explicit_quat) # get a euler
        self.heading = euler[2] # and update the heading

        self.acceleration = data.linear_acceleration.x
        self.velocity = self.prev_acceleration + self.acceleration * 0.04 # "imu refreshes at ~25Hz" so 0.04
        self.prev_acceleration = self.acceleration # so, you know, we actually update

    def getVelocity(self):
        return ((self.c_velocity * 2) + (self.v_velocity * 2) + self.a_velocity) / 5 # average velocity, with less weight
        # given to velocity got from acceleration
    
    # ===publishing===
    # and publish!
    def getMessage(self):
        positionMessage = Position()
        positionMessage.x = self.x
        positionMessage.y = self.y
        positionMessage.heading = self.heading
        return positionMessage