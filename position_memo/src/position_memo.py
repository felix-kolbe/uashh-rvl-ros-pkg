#!/usr/bin/env python  
import roslib
roslib.load_manifest('position_memo')
import rospy
import tf
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion

import sys, select, termios, tty
import string
import math, random

TAU = math.pi*2

msg = """
Position memo tool commands:
---------------------------
h : show this help
q : quit (^C)

l : list stored positions
p : print current position
s : store current position

r : read positions from file
w : write positions to file

g : go to selectable position
r : go to a random position within robot's radius
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def getCurrentPose(listener):
    try:
        trans,rot = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException):
        return 0
    #print "translational: %s\nrotational: %s" % (trans, rot)
    pose = Pose(Point(*trans), Quaternion(*rot))
    return pose


positions = {'zero': Pose()}
positions['zero'].orientation.w = 1


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('position_memo')

    print 'Intializing transform listener and move base action client.'
    listener = tf.TransformListener()
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    
    print msg
    
    while not rospy.is_shutdown():
        key = getKey()
        
        ## quit
        if key == '\x03' or key == 'q':
            break
        
        ## help
        elif key == 'h':
            print msg
        
        ## list positions
        elif key == 'l':
            for k, v in positions.iteritems():
                print k, '\n', v
        
        ## current position
        elif key == 'p':
            pose = getCurrentPose(listener)
            print 'Current position:', pose
            
        ## store position
        elif key == 's':
            pose = getCurrentPose(listener)
            print 'Current position:', pose
            #print "translational: %s rotational: %s" % (trans, rot)
            posname = raw_input('enter name to store position: ')
            if len(posname) == 0:
                print 'No input, nothing stored'
            else:
                positions[posname] = pose
            
        ## go to position
        elif key == 'g':
            print 'Stored position names:'
            for k, v in positions.iteritems():
                print k
            
            posname = raw_input('enter position name to go to: ')
            if posname not in positions:
                continue
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "/map"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            goal.target_pose.pose = positions[posname]
            
            print 'Waiting for base to reach goal...'
            client.send_goal_and_wait(goal)
            #client.wait_for_result(rospy.Duration.from_sec(25.0))
            
            if(client.get_state() == actionlib.simple_action_client.SimpleGoalState.DONE):
                print ("Hooray, the base reached the goal.")
            else:
                print ("The base failed to move to the goal for some reason")
            
        ## go to random position
        elif key == 'r':
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "/base_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            radius = random.random()*2 + 1  # 1-3 m
            #yaw = random.random()*TAU/2 - TAU/4    # +-90 deg
            yaw = random.random()*TAU*3/4 - TAU*3/8    # +-135 deg
            
            quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
            goal.target_pose.pose.orientation = Quaternion(*quat)
            goal.target_pose.pose.position = Point(math.cos(yaw)*radius, math.sin(yaw)*radius, 0)
            print 'Random goal:'
            print goal
            
            print 'Waiting for base to reach goal...'
            client.send_goal_and_wait(goal)
            #client.wait_for_result(rospy.Duration.from_sec(25.0))
            
            if(client.get_state() == actionlib.simple_action_client.SimpleGoalState.DONE):
                print ("Hooray, the base reached the goal.")
            else:
                print ("The base failed to move to the goal for some reason. State: %d, %s" % (client.get_state(), client.get_goal_status_text()) )
        
        ## skip delimiter lines if bad keys pressed
        else:
            continue
            
        print '---------------------'
        
    ## end while
