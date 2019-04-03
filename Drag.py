#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry


Linear_Drag_X = 0
Linear_Drag_Y = 0
Velocity1 = 0
Velocity2 = 0




def get_velocity1(data):
	global Velocity1
	Velocity1 = data.twist.twist.linear.x



def get_velocity2(data):
	global Velocity2
	Velocity2 = data.twist.twist.linear.x
	
	
	#Linear_Drag_X = (10 / Max_Velocity)
	
	

def Print_Linear_Drag():
	rospy.loginfo('Linear Drag X:  {}, Linear Drag Y:  {}'.format(Linear_Drag_X,Current_VelocityX))



def Apply_Force():
	pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=20)
	rospy.init_node('move_sub', anonymous=False)
	force_msg = WrenchStamped()
	force_msg.wrench.force.x = 10
	
	
	



	while not rospy.is_shutdown():
		rospy.Subscriber("/odom", Odometry, get_velocity1)
		velocity1 = Velocity1
		
		rospy.Subscriber("/odom", Odometry, get_velocity2)
		velocity2 = Velocity2
		Compare_Velocities = abs(velocity1 - velocity2)
		rospy.loginfo('Velocity1: {},Velocity2: {}, Compare Velocities: {}'.format(velocity1, velocity2, Compare_Velocities))
		while (True):
			pub.publish(force_msg)
			if Compare_Velocities == 0:
				break
		force_stop = WrenchStamped()
		force_stop.wrench.force.x = -10
		pub.publish(force_stop)
			
		#Print_Linear_Drag()
		
		
		
		
		

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
