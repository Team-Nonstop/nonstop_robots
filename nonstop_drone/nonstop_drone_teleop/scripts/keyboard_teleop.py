#!/usr/bin/env python
import roslib; roslib.load_manifest('nonstop_drone_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:

   U    I    O
      J    L
        ,     

q/z : increase/decrease x speeds by 10%
w/x : increase/decrease y linear speed by 10%
e/c : increase/decrease z angular speed by 10%
r/v : increase/decrease z angular speed by 10%

anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'i':(1,0,0,0),
		',':(-1,0,0,0),
		'l':(0,-1,0,0),
		'j':(0,1,0,0),
		'p':(0,0,1,0),
		';':(0,0,-1,0),
		'u':(0,0,0,-1),
		'o':(0,0,0,1),
	       }

speedBindings={
		'q':(1.1, 1.1, 1.1, 1),
		'z':(0.9, 0.9, 0.9, 1),
		'w':(1.1, 1, 1, 1),
		'x':(0.9, 1, 1, 1),
		'e':(1, 1.1, 1, 1),
		'c':(1, 0.9, 1, 1),
		'r':(1, 1, 1.1, 1),
		'v':(1, 1, 0.9, 1),
		't':(1, 1, 1, 1.1),
		'b':(1, 1, 1, 0.9),
	      }

commandBindings={
		's':(0),
		'd':(1),
		'1':(2),
		'g':(3),
				}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed_x = 0.1
speed_y = 0.1
speed_z = 0.5
speed_yaw = 2


if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist)

	land_pub = rospy.Publisher('/ardrone/land', Empty)
	reset_pub = rospy.Publisher('/ardrone/reset', Empty)
	toggle_pub = rospy.Publisher('/ardrone/togglecam', Empty)
	takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty)
	rospy.init_node('ardrone_teleop')

	x = 0
	y = 0
	z = 0
	yaw = 0
	status = 0

	try:
		print msg
		while(1):
			key = getKey()

			if key in moveBindings.keys():
				x = moveBindings[key][0]
				y = moveBindings[key][1]
				z = moveBindings[key][2]
				yaw = moveBindings[key][3]
			elif key in speedBindings.keys():
				speed_x = speed_x * speedBindings[key][0]
				speed_y = speed_y * speedBindings[key][1]
				speed_z = speed_z * speedBindings[key][2]
				speed_yaw = speed_yaw * speedBindings[key][3]



			else:
				x = 0
				y = 0
				z = 0
				yaw = 0

				if (key == 'a'):
					land_pub.publish(Empty())
					print "Landing"
				if (key == 's'):
					takeoff_pub.publish(Empty())
					print "Takeoff"
				if (key == 'd'):
					reset_pub.publish(Empty())
					print "Reset"
				if (key == 'f'):
					toggle_pub.publish(Empty())
					print "ToggleCam"
				if (key == '\x03'):
					break

			twist = Twist()
			twist.linear.x = x*speed_x; twist.linear.y = y*speed_y; twist.linear.z = z*speed_z
			twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = yaw*speed_yaw
			pub.publish(twist)

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


