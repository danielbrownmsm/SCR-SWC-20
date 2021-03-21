from swc_msgs.msg import Control

class ControlHandler:
    def __init__(self):
        self.msg = Control()
        self.msg.speed = 0
        self.msg.turn_angle = 0

        self.isBumping = False     
    
    def bumperCallback(self, data):
        self.isBumping = data.data
    
    def pathCallback(self, data):
        self.msg.speed = data.speed
        self.msg.turn_angle = data.turn_angle
    
    def visionCallback(self, data):
        # handle vision corrections
        pass
    
    def getMessage(self):
        if self.isBumping: # If we're touching something
            self.msg.speed = -8 # HARD REVERSE!
            self.msg.turn_angle = 0

        return self.msg