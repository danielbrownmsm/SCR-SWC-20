#!/usr/bin/env python

import rospy
from Util import PIDController, PurePursuit, dist
from swc_msgs.msg import State, Obstacles, Control
from swc_msgs.srv import Waypoints

class ControlHandler:
    def __init__(self, waypoints):
        self.state = State() # just the default values
        self.obstacles = []
        self.distancePID = PIDController(3, 0, 0.5) #TODO tune
        self.anglePID = PIDController(0.2, 0.0001, 0.001) #TODO tune
        self.path = waypoints # path will later hold additional points as needed to account for obstacles
        self.pure_pursuit = PurePursuit(waypoints, 1) #TODO tune lookahead dist
    
    def stateCallback(self, data):
        self.state = data # HACK this probably doesn't work
    
    def obstacleCallback(self, data):
        pass
    
    def getMessage(self):
        msg = Control()
        msg.speed = self.distancePID.calculate(dist(self.state.x, self.state.y, *self.pure_pursuit.getGoalPoint())) # wait I can do asterik? Python is freakin' awesome
        self.anglePID.setSetpoint(self.pure_pursuit.getNextHeading(self.state))
        msg.angle = self.anglePID.calculate(self.state.angle)
        
        return msg


def main():
    global controlHandler
    global publisher

    # Initalize our node in ROS
    rospy.init_node("control_node")
    print("Control node initialized!")

    # Create a Publisher that we can use to publish messages to the /daniel/state topic
    publisher = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    rospy.wait_for_service("/sim/waypoints")
    waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()
    print("Waypoints aquired!")
    
    controlHandler = ControlHandler(waypoints)
    
    rospy.Subscriber("/daniel/state", State, controlHandler.stateCallback)
    rospy.Subscriber("/daniel/obstacles", Obstacles, controlHandler.obstacleCallback)

    rospy.Timer(rospy.Duration(0.1), publish)

    print("Control node setup complete")

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

publisher = None
controlHandler = None

def publish(event):
    # Publish the message to /daniel/state so the simulator receives it
    global publisher
    global controlHandler

    publisher.publish(controlHandler.getMessage())


# so if anything imports our code (documentation tools, linters, etc.) it isn't run automatically and things don't get broken
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass