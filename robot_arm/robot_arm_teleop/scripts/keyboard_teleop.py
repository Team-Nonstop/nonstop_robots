#!/usr/bin/env python
import roslib; roslib.load_manifest('robot_arm_teleop')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:

  [  ]


q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
anything else : stop

CTRL-C to quit
"""

moveBindings = {
		'[':(1),
		']':(-1),
	       }

speedBindings={
		'q':(1.1),
		'z':(0.9),
	      }

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


angle = 180.
speed = 1.

def angles(angle):
	return "currently:\tangle %s" % (angle)

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	

	pub_angle = rospy.Publisher('motor_angle', Float32)
	rospy.init_node('keyboard_teleop')

	x = 0.
	status = 0

	try:
		print msg
		print angles(angle)
		while(1):
			key = getKey()
			if key in moveBindings.keys():
				x = x+moveBindings[key]
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key]

				
				if (status == 14):
					print msg
				status = (status + 1) % 15
			else:
				x = 0
				if (key == '\x03'):
					break


			angle = Float32()
			
			angle = x*speed;

			pub_angle.publish(angle)
		

	finally:
		angle = Float32()
		angle = 180.
		pub_angle.publish(angle)

    		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


