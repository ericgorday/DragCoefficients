#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
# Initialized ros parameters and global values (feel free to tweak, values were empirically gained through trial/error)
linear_drag_x = 0
linear_drag_y = 0
linear_drag_z = 0
roll_drag = 0
pitch_drag = 0
yaw_drag = 0
velocity = 0
'''
Velocities between two time intervals will never be truly 'equal' in real world
Must compare the abs of the difference of the two with a small delta value
if the difference is less than this value we can conclude that the numbers are satisfyingly close
also used as a value that is satisfyingly close to zero. Hard-coded (can be changed).
'''
delta = .00001
# Max velocity cannot be initialized to 0 or there will be an initial divide-by-zero error
max_velocity = .1
bouyancy = 0
# Magnitude of applied force for determining drag coeffcients in linear axes
applied_linear_force = rospy.get_param("linear_force", default=10)
# Magnitude of applied torque for determining drag coeffcients in rotational axes
applied_torque = rospy.get_param("torque", default=5)
applied_force_down = rospy.get_param('force_down', default=-20)
time_of_apllied_force_down = rospy.get_param('time_of_force_down', default=10)


def find_bouyancy(choice):  # Initial function used to upward force of buoyancy (used in determining drag in Z axis)
    global bouyancy
    down_force = -.5  # Applies initial downward force in z axis
    pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=30)
    rospy.init_node('move_sub', anonymous=False)
    force_msg = WrenchStamped()
    force_msg.wrench.force.z = applied_force_down
    pub.publish(force_msg)  # publish wrench with force
    rospy.loginfo('Moving sub down...')
    rospy.sleep(time_of_apllied_force_down)  # node sleeps for some amount of time before continuing
    force_msg.wrench.force.z = 0  # Applies 0 force which stops downward force
    pub.publish(force_msg)
    while not (rospy.is_shutdown()):
        rospy.Subscriber("/odom", Odometry, get_velocity, choice)
        rospy.sleep(1)
        if velocity > 0.0:
            rospy.loginfo('Appling a force down to calculate the bouyancy')
            while not (rospy.is_shutdown()):
                force_msg.wrench.force.z = down_force
                pub.publish(force_msg)
                down_force = down_force - .001
                rospy.sleep(.01)
                if velocity < 0.0:
                    break
            rospy.loginfo('bouyancy found!')
            break
    bouyancy = abs(down_force)
    rospy.loginfo('Bouyancy: {}'.format(bouyancy))


def get_velocity(data, choice):  # Function sets velocity in a certain axis depending on char input
    global velocity
    # linear
    if choice == 'x':
        velocity = data.twist.twist.linear.x
    elif choice == 'y':
        velocity = data.twist.twist.linear.y
    elif choice == 'z':
        velocity = data.twist.twist.linear.z
    # rotational
    elif choice == 'rl':
        velocity = data.twist.twist.angular.x
    elif choice == 'p':
        velocity = data.twist.twist.angular.y
    elif choice == 'yw':
        velocity = data.twist.twist.angular.z


def calculate_drag(choice):
    '''
    Calculates drag based on the initial applied force and the approximate max velocity
    the sub achieves in that axis. See for formula: http://hyperphysics.phy-astr.gsu.edu/hbase/airfri.html
    Axis determined by char argument.
    '''
    global linear_drag_x, linear_drag_y, linear_drag_z, pitch_drag, roll_drag, yaw_drag, max_velocity, bouyancy
    if (choice == 'x'):
        linear_drag_x = (applied_linear_force / abs(max_velocity))
    elif (choice == 'y'):
        linear_drag_y = (applied_linear_force / abs(max_velocity))
    elif (choice == 'z'):
        # Buoyancy affects z axis and must be subtracted from applied for before division.
        linear_drag_z = ((applied_linear_force - bouyancy) / (abs(max_velocity)))
    elif (choice == 'rl'):
        roll_drag = (applied_torque / abs(max_velocity))
    elif (choice == 'p'):
        pitch_drag = (applied_torque / abs(max_velocity))
    elif (choice == 'yw'):
        yaw_drag = (applied_torque / abs(max_velocity))


def apply_force(choice):
    '''
    Function applies force in a given axis and allows sub to achieve
    terminal (linear/rotationl) velocity in that direction. Once that
    max velocity is found, it is used in the calculate_drag() function.
    '''
    global max_velocity, bouyancy, linear_drag_z
    pub = rospy.Publisher('/wrench', WrenchStamped, queue_size=20)  # Publisher for applying wrench (force/torque)
    rospy.init_node('move_sub', anonymous=False)
    force_msg = WrenchStamped()
    # Char argument determines axis of applied force/torque
    if(choice == 'x'):
        force_msg.wrench.force.x = applied_linear_force
    elif(choice == 'y'):
        force_msg.wrench.force.y = applied_linear_force
    elif(choice == 'z'):
        force_msg.wrench.force.z = -applied_linear_force
    elif(choice == 'yw'):
        force_msg.wrench.torque.z = applied_torque
    elif(choice == 'rl'):
        force_msg.wrench.torque.x = applied_torque
    elif(choice == 'p'):
        force_msg.wrench.torque.y = applied_torque

    pub.publish(force_msg)
    rospy.loginfo('Appling a force to find the drag in the {} direction'.format(choice))
    while not (rospy.is_shutdown()):
        '''
        On each iteration of while loop: velocity is gained at two points in time (deltat = 2 seconds)
        the velocity is compared by taking the absolute value of the difference of the two velocities
        if the abs of the difference of the two velocities is smaller than small delta value,
        the two velocities are assumed to be approximately equal. If two velocities taken at different
        time intervals are equal, the velocity is assumed to be maximized, reaching terminal velocity.
        '''
        rospy.Subscriber("/odom", Odometry, get_velocity, choice)
        velocity1 = velocity
        rospy.sleep(2)
        velocity2 = velocity
        Compare_Velocities = abs(velocity1 - velocity2)
        if Compare_Velocities < delta:
            rospy.loginfo('Velocities are equal')
            max_velocity = velocity2
            calculate_drag(choice)  # Once velocity is max, calculate drag using max velocity
            rospy.loginfo('Linear Drag Values-- x: {},  y: {},  z: {}'.format(linear_drag_x, linear_drag_y, linear_drag_z))
            rospy.loginfo('Rotational Drag Values-- yaw: {},  pitch: {},  roll: {}'.format(yaw_drag, pitch_drag, roll_drag))
            break  # When the velocities are 'equal', the loop breaks.

    # Once drag is calculated, apply wrench with force/torque of zero to slow sub in that axis
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
    rospy.loginfo('stopping sub')
    while(velocity > delta and not rospy.is_shutdown()):
        # While loop stops program from proceeding until the sub has basically stopped movement in that direction
        rospy.sleep(2)
        continue


if __name__ == '__main__':
    try:
        # Buoyancy is calculated
        find_bouyancy('z')
        # Drag Coeffcients are calculated in each axis
        apply_force('z')
        apply_force('y')
        apply_force('x')
        apply_force('yw')
        apply_force('rl')
        apply_force('p')
        # Drag coefficients are written to file, do with them what you wish.
        data = {'x': linear_drag_x, 'y': linear_drag_y, 'z': linear_drag_z, 'yaw': yaw_drag, 'pitch': pitch_drag, 'roll': roll_drag}
        f = open('drag_coefficients.yaml','w')
        yaml.dump(data,f)
    except rospy.ROSInterruptException as e:
        rospy.logerr(e)

