class ControlHandler:
	def __init__(self):
		pass
	
	def stateCallback(self):
		pass
	
	def obstacleCallback(self):
		pass
	
	def getMessage(self):
		pass

controlHandler = ControlHandler()

def main():
    global controlHandler
    global publisher

    # Initalize our node in ROS
    rospy.init_node("fusion_node")
    print("Fusion node initialized!")

    # Create a Publisher that we can use to publish messages to the /daniel/state topic
    publisher = rospy.Publisher("/sim/control", Control, queue_size=1)

    # Wait for Waypoints service and then request waypoints
    #rospy.wait_for_service("/sim/waypoints")
    #waypoints = rospy.ServiceProxy("/sim/waypoints", Waypoints)()
    print("Waypoints aquired!")
	#TODO add PurePursuit and stuff

    rospy.Subscriber("/daniel/state", State, controlHandler.stateCallback)
    rospy.Subscriber("/daniel/obstacles", Obstacles, controlHandler.obstacleeCallback)

    # Create a timer that calls timer_callback() with a period of 0.1, because most of our sensors update at 10 Hz
    rospy.Timer(rospy.Duration(0.1), publish)

    print("Control node setup complete")

    # Let ROS take control of this thread until a ROS wants to kill
    rospy.spin()

def publish(event):
    # Publish the message to /daniel/state so the simulator receives it
    global publisher
    global controlHandler

    publisher.publish(controlHandler.getState())


# so if anything imports our code (documentation tools, linters, etc.) it isn't run automatically and things don't get broken
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass