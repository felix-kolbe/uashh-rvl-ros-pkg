#!/usr/bin/env python

""" This file generates easy to use smach states needed to move the robot base. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf
import math

#import smach
#import smach_ros
from smach import State
from smach_ros import ServiceState, SimpleActionState

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion

    
def getMoveBaseGoalInMapState(x, y):
    return getMoveBaseGoalState("/map", x, y)

def getMoveBaseGoalInOdomState(x, y):
    return getMoveBaseGoalState("/odom", x, y)


def getMoveBaseGoalState(frame, x=0, y=0, yaw=0):
    base_goal = MoveBaseGoal()
    base_goal.target_pose.header.frame_id = frame
        
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    base_goal.target_pose.pose.orientation = Quaternion(*quat)
    base_goal.target_pose.pose.position = Point(x, y, 0)
    
    return SimpleActionState('move_base', 
                             MoveBaseAction, 
                             goal=base_goal
                             )
    
