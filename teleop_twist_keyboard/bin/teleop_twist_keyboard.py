#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
Reading from the keyboard and publishing to twist!
---------------------------
Moving around:    On Numpad:     Meaning:
   u   i   o      7   8   9      <^  ^  ^>
   j   k   l      4   5   6      <   o   >
   m   ,   .      1   2   3      <v  v  v>

q|+/z|y|- : increase/decrease both speeds by 10%
w/x       : increase/decrease linear speed by 10%
e/c       : increase/decrease angular speed by 10%
anything else : stop

CTRL-C : stop and quit
"""

moveBindings = {# (translational x, rotational z)
		'i':(1,0),
		'8':(1,0),
		'o':(1,-1),
		'9':(1,-1),
		'j':(0,1),
		'4':(0,1),
		'l':(0,-1),
		'6':(0,-1),
		'u':(1,1),
		'7':(1,1),
		',':(-1,0),
		'2':(-1,0),
		'.':(-1,1),
		'3':(-1,1),
		'm':(-1,-1),
		'1':(-1,-1),
		}

speedBindings={# (translational x, rotational z)
		'q':(1.1,1.1),
		'+':(1.1,1.1),
		'z':(.9,.9),
		'y':(.9,.9),
		'-':(.9,.9),
		'w':(1.1,1),
		'x':(.9,1),
		'e':(1,1.1),
		'c':(1,.9),
		}

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

speed = 0.2 # .5
turn = 0.5 # 1

def vels(speed,turn):
	#return "currently:\tspeed %s\tturn %s " % (speed,turn)
	return "current speeds:\ntranslational:\t%.3f\nrotational:\t%.3f " % (speed,turn)

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
	pub = rospy.Publisher('cmd_vel', Twist)
	rospy.init_node('teleop_twist_keyboard')

	x = 0
	th = 0
	status = 0

	try:
		print msg
		print vels(speed,turn)
		while not rospy.is_shutdown():
			key = getKey()
			
			## quit
			if key == '\x03':
				break
			
			## change speed
			elif key in speedBindings.keys():
				speed = speed * speedBindings[key][0]
				turn = turn * speedBindings[key][1]
				print msg
				print vels(speed,turn)
#				if (status == 14):
#					print msg
#				status = (status + 1) % 15

			## move or anykey-stop
			else:
				if key in moveBindings.keys():
					x = moveBindings[key][0]
					th = moveBindings[key][1]
				else:
					x = 0
					th = 0

				twist = Twist()
				twist.linear.x = x*speed; twist.linear.y = 0; twist.linear.z = 0
				twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
				pub.publish(twist)

	except Exception, e:
		print e

	finally:
		print 'Sending a zero velocity message...'
		twist = Twist()
		twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
		twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
		pub.publish(twist)

		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


