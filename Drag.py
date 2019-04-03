#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry


Linear_Drag_X = 0
Linear_Drag_Y = 0
Velocity = 0





def get_velocity(data):
	global Velocity
	Velocity = data.twist.twist.linear.x




	
	
	#Linear_Drag_X = (10 / Max_Velocity)
	
	

def Print_Linear_Drag():
	rospy.loginfo('Linear Drag X:  {}, Linear Drag Y:  {}'.format(Linear_Drag_X,Current_VelocityX))



def Apply_Force():
	pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=20)
	rospy.init_node('move_sub', anonymous=False)
	force_msg = WrenchStamped()
	force_msg.wrench.force.x = 10
	pub.publish(force_msg)
	
	



	while (True):
		rospy.Subscriber("/odom", Odometry, get_velocity)
		velocity1 = Velocity
		rospy.sleep(2)
		velocity2 = Velocity
		Compare_Velocities = abs(velocity1 - velocity2)
		rospy.loginfo('Velocity1: {},Velocity2: {}, Compare Velocities: {}'.format(velocity1, velocity2, Compare_Velocities))
		if Compare_Velocities == 0:
			print("Velocities are equal")
			break
		


	force_stop = WrenchStamped()
	force_stop.wrench.force.x = 0
	pub.publish(force_stop)
	while(Velocity != 0):
		continue
	Print_Linear_Drag()
		
		
		
		
		

if __name__== '__main__':
	try:
		Apply_Force()
	except rospy.ROSInterruptException:
		pass
		
		
		
		

if __name__== '__main__':
	try:
		Apply_Force()
	except rospy.ROSInterruptException:
		pass
