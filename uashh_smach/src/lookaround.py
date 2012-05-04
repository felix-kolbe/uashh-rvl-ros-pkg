#!/usr/bin/env python

""" This files generates poses which let the robot look around and let him scan his environment.
These movements via MoveArm.py are provided as a state machine container.
"""

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import smach
import smach_ros
from smach import State, StateMachine, Sequence
from smach_ros import ServiceState, SimpleActionState

import MoveArm





""" calculating lookaround poses """

poses = [[0,0,0,0,0]]

#    poses_lookaround_turnings = [3, 2, 1, 0, -1, -2]
poses_lookaround_turnings = [1, 0, -1]
poses_lookaround = [ # without first joint!
#                    [0.523,1.309,-1.919,0], # looking down
#                    [0.523,1.047,-1.745,0], # a bit higher
                    [0.523,0.689,-1.745,0]  # a bit below horizontally
                    ]
 
for pose in poses_lookaround:
    for turn in poses_lookaround_turnings:
        poses.append([turn]+pose)
        
    poses_lookaround_turnings.reverse()

poses.append([0,0,0,0,0])

#print poses


def get_lookaround_smach():

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')
    
    for i in range(len(poses)):
        sq.add('MOVE_ARM_LOOKAROUND_%d'%i,
               MoveArm.getMoveArmToJointsSimpleActionState(poses[i])
               );
    
    return sq

