#!/usr/bin/env python
import math
import rospy
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry


Linear_Drag_X = 0
Linear_Drag_Y = 0
Linear_Drag_Z = 0
Roll_Drag = 0
Pitch_Drag = 0
Yaw_Drag = 0
Velocity = 0
Max_Velocity = .1

def get_velocity(data,choice):
	global Velocity, Linear_Drag_X,Linear_Drag_Y ,Linear_Drag_Z,Roll_Drag,Pitch_Drag,Yaw_Drag, Max_Velocity
	if choice == 'x':
		Velocity = data.twist.twist.linear.x
	elif choice == 'y':
		Velocity = data.twist.twist.linear.y
	elif choice == 'z':
		Velocity = data.twist.twist.linear.z
	elif choice == 'rl':
		Velocity = data.twist.twist.angular.x
	elif choice == 'p':
		Velocity = data.twist.twist.angular.y
	elif choice == 'yw':
		Velocity = data.twist.twist.angular.z
	

def Print_Linear_Drag():
	rospy.loginfo('Linear Drag X:  {}, Linear Drag Y:  {}, Linear Drag z: {}, Linear Drag rl: {}, Linear Drag yw: {}, Linear Drag Pitch: {}'.format(Linear_Drag_X,Linear_Drag_Y, Linear_Drag_Z, Roll_Drag, Yaw_Drag, Pitch_Drag))


def Calculate_Drag(choice):
	global Linear_Drag_X, Linear_Drag_Y, Linear_Drag_Z, Pitch_Drag, Roll_Drag, Yaw_Drag, Max_Velocity

	if (choice == 'x'):
		Linear_Drag_X = (10 / abs(Max_Velocity))
	elif (choice == 'y'):
		Linear_Drag_Y = (10 / abs(Max_Velocity))
	elif (choice == 'z'):
		Linear_Drag_Z = (10 / abs(Max_Velocity))
	elif (choice == 'rl'):
		Roll_Drag = (5 / abs(Max_Velocity))
	elif (choice == 'p'):
		Pitch_Drag = (5 / abs(Max_Velocity))
	elif (choice == 'yw'):
		Yaw_Drag = (5 / abs(Max_Velocity))


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
		force_msg.wrench.force.z = -10
	elif(choice == 'yw'):
		force_msg.wrench.torque.y = 5
	elif(choice == 'rl'):
		force_msg.wrench.torque.x = 5
	elif(choice == 'p'):
		force_msg.wrench.torque.z = 5
	pub.publish(force_msg)
	
	



	while (True):
		rospy.Subscriber("/odom", Odometry, get_velocity, choice)
		velocity1 = Velocity
		rospy.sleep(2)
		velocity2 = Velocity
		Compare_Velocities = abs(velocity1 - velocity2)
		rospy.loginfo('Velocity1: {},Velocity2: {}, Compare Velocities: {}'.format(velocity1, velocity2, Compare_Velocities))
		if Compare_Velocities < .000001:
			Max_Velocity = velocity2
			print("Velocities are equal")
			Calculate_Drag(choice)
			break
		
	if(choice == 'x'):
		force_msg.wrench.force.x = 0
	elif(choice == 'y'):
		force_msg.wrench.force.y = 0
	elif(choice == 'z'):
		force_msg.wrench.force.z = 0
	elif(choice == 'yw'):
		force_msg.wrench.torque.z = 0
	elif(choice == 'rl'):
		force_msg.wrench.torque.x = 0
	elif(choice == 'p'):
		force_msg.wrench.torque.y = 0
	

	pub.publish(force_msg)
	while(Velocity > .0001):
		continue
	Print_Linear_Drag()
		
		
		
		
		

if __name__== '__main__':
	try:
		Apply_Force('z')
		Apply_Force('y')
		Apply_Force('x')
		Apply_Force('yw')
		Apply_Force('rl')
		Apply_Force('p')
		file_object = open("DragCoefficients", 'w')
		file_object.write("Drag Coefficients:")
		file_object.write("\nLinear X: " + str(Linear_Drag_X))
		file_object.write("\nLinear Y: " + str(Linear_Drag_Y))
		file_object.write("\nLinear Z: " + str(Linear_Drag_Z))
		file_object.write("\nRoll: " + str(Roll_Drag))
		file_object.write("\nPitch: " + str(Pitch_Drag))
		file_object.write("\nYaw: " + str(Yaw_Drag))
		file_object.close()
	except rospy.ROSInterruptException:
		pass
