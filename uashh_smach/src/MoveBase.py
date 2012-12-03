#!/usr/bin/env python

""" This file generates easy to use smach states needed to move the robot base. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf
import math
import random

#import smach
#import smach_ros
from smach import State
from smach_ros import ServiceState, SimpleActionState

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion


TAU = math.pi*2   # one tau is one turn. simply as that.

    
def getMoveBaseGoalInMapState(x, y):
    return getMoveBaseGoalState("/map", x, y)

def getMoveBaseGoalInOdomState(x, y):
    return getMoveBaseGoalState("/odom", x, y)


def getMoveBaseGoalState(frame, x=0, y=0, yaw=0):
    base_goal = MoveBaseGoal()
    base_goal.target_pose.header.frame_id = frame
    base_goal.target_pose.header.stamp = rospy.Time.now()
        
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    base_goal.target_pose.pose.orientation = Quaternion(*quat)
    base_goal.target_pose.pose.position = Point(x, y, 0)
    
    return SimpleActionState('move_base', 
                             MoveBaseAction, 
                             goal=base_goal
                             )
    

def getMoveRandomGoalState():
    radius = random.random()*2 + 1  # 1-3 m
    #yaw = random.random()*TAU/2 - TAU/4    # +-90 deg
    yaw = random.random()*TAU*3/4 - TAU*3/8    # +-135 deg
    
    return getMoveBaseGoalState("/base_link", math.cos(yaw)*radius, math.sin(yaw)*radius, yaw)
    


MINIMUM_MOVED_DISTANCE = 1.5



class HasMovedState(smach.State):
    
    def _getXY(self):
        try:
            trans,rot = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            return trans[0], trans[1]
        except (tf.LookupException, tf.ConnectivityException) as e:
            print e
            return 0,0
    
    def __init__(self, minimumDistance):
        smach.State.__init__(self, outcomes=['movement_exceeds_distance', 'movement_within_distance'])
        self.minimumDistance = minimumDistance
        self.listener = tf.TransformListener();
        self.lastX, self.lastY = self._getXY()

    def execute(self, userdata):
        currentX, currentY = self._getXY()
        currentDistance = math.sqrt(math.pow(currentX, 2) + math.pow(currentY, 2))
        rospy.logdebug("currentXY: %f,%f lastXY: %f,%f currentDistance: %f minimumDistance: %f", self.lastX, self.lastY, currentX, currentY, currentDistance, self.minimumDistance)
        if currentDistance >= self.minimumDistance:
            self.lastX = currentX
            self.lastY = currentY
            return 'movement_exceeds_distance'
        else:
            return 'movement_within_distance'
    
    
