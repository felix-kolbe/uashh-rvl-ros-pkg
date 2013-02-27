#!/usr/bin/env python
import roslib; roslib.load_manifest('move_demo')
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


speed = 0.2 # meters/sec
distance = 1 # meters


currentOdometry = None
def odometryUpdate(newOdometry):
	global currentOdometry
	currentOdometry = newOdometry


if __name__=="__main__":
	try:
		sub = rospy.Subscriber('odom', Odometry, odometryUpdate)	
		pub = rospy.Publisher('cmd_vel', Twist)
		rospy.init_node('move_demo')

		print 'Waiting for odometry...'
		while currentOdometry == None:
			rospy.sleep(0.1)
		startOdometry = currentOdometry
		
		print 'Starting robot movement...'
		deltaX = deltaY = 0
		while (deltaX*deltaX + deltaY*deltaY) < distance*distance:
			twist = Twist()
			twist.linear.x = speed
			pub.publish(twist)
			
			rospy.sleep(0.1)
			
			deltaX = currentOdometry.pose.pose.position.x - startOdometry.pose.pose.position.x
			deltaY = currentOdometry.pose.pose.position.y - startOdometry.pose.pose.position.y
		print 'Robot reached distance...'

	except Exception, e:
		print e

	finally:
		print 'Sending a zero velocity message...'
		twist = Twist()
		pub.publish(twist)
