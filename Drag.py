#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry


Linear_Drag_X = 0
Linear_Drag_Y = 0
Velocity = 0
Max_Velocity = .1





def get_velocity(data,choice):
	global Velocity, Linear_Drag_X, Max_Velocity
	Velocity = data.twist.twist.linear.x
	Linear_Drag_X = (10 / Max_Velocity)
	
	

def Print_Linear_Drag():
	rospy.loginfo('Linear Drag X:  {}, Linear Drag Y:  {}'.format(Linear_Drag_X,0))



def Apply_Force(choice):
	global Max_Velocity
	pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=20)
	rospy.init_node('move_sub', anonymous=False)
	force_msg = WrenchStamped()
	if(choice == 'x'):
		force_msg.wrench.force.x = 10
	elif(choice == 'y'):
		force_msg.wrench.force.y = 10
	elif(choice == 'z'):
		force_msg.wrench.force.z = 10
	elif(choice == 'yw'):
		force_msg.wrench.angular.y = 10
	elif(choice == 'rl'):
		force_msg.wrench.angular.x = 10
	elif(choice == 'p'):
		force_msg.wrench.angular.z = 10
	pub.publish(force_msg)
	
	



	while (True):
		rospy.Subscriber("/odom", Odometry, get_velocity(choice))
		velocity1 = Velocity
		rospy.sleep(2)
		velocity2 = Velocity
		Compare_Velocities = abs(velocity1 - velocity2)
		rospy.loginfo('Velocity1: {},Velocity2: {}, Compare Velocities: {}'.format(velocity1, velocity2, Compare_Velocities))
		if Compare_Velocities < .000001:
			Max_Velocity = velocity2
			print("Velocities are equal")
			break
		
	if(choice == 'x'):
		force_msg.wrench.force.x = 0
	elif(choice == 'y'):
		force_msg.wrench.force.y = 0
	elif(choice == 'z'):
		force_msg.wrench.force.z = 0
	elif(choice == 'yw'):
		force_msg.wrench.angular.y = 0
	elif(choice == 'rl'):
		force_msg.wrench.angular.x = 0
	elif(choice == 'p'):
		force_msg.wrench.angular.z = 0
	



	pub.publish(force_msg)
	while(Velocity > .0000001):
		continue
	










	Print_Linear_Drag()
		
		
		
		
		

if __name__== '__main__':
	try:
		Apply_Force()
	except rospy.ROSInterruptException:
		pass
