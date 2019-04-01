#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry


Linear_Drag_X = 0
Linear_Drag_Y = 0
Max_Velocity = 0
Current_VelocityX = 0
Compare_Velocties = math.abs(Max_Velocity - Current_VelocityX)



def callback(data):
	global Linear_Drag_X, Max_Velocity, Current_VelocityX
	Current_VelocityX = data.twist.twist.linear.x
	
	
	if Max_Velocity < Current_VelocityX:
		Max_Velocity = Current_VelocityX
	#else:
	#	force_msg.wrench.force.x = 0
	#	pub.publish(force_msg)
	Linear_Drag_X = (10 / Max_Velocity)
	
	

def Print_Linear_Drag():
	rospy.loginfo('Linear Drag X:  {}, Linear Drag Y:  {}'.format(Linear_Drag_X,Linear_Drag_Y))



def Apply_Force():
	pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=20)
	rospy.init_node('move_sub', anonymous=False)
	force_msg = WrenchStamped()
	force_msg.wrench.force.x = 10
	
	
	



	while not rospy.is_shutdown():
		if Compare_Velocties != .000001:
			pub.publish(force_msg)
			rospy.Subscriber("/odom", Odometry, callback)
		else:
			force_stop = WrenchStamped()
			force_stop.wrench.force.x = 0
			
		Print_Linear_Drag()
		
		
		
		
		

if __name__== '__main__':
	try:
		Apply_Force()
	except rospy.ROSInterruptException:
		pass