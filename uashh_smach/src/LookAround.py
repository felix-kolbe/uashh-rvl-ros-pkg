#!/usr/bin/env python

""" This files generates poses which let the robot look around and let him scan his environment.
These movements via MoveArm.py are provided as a state machine container.
"""

import roslib; roslib.load_manifest('uashh_smach')

import rospy
#import smach
#import smach_ros
from smach import State, StateMachine, Sequence
from smach_ros import ServiceState, SimpleActionState

import MoveArm





""" calculating lookaround poses """

#pose_default = [0,0,0,0,0]
pose_default = [0,0.3,0.3,-0.6,0]
poses = [pose_default]

poses_lookaround_turnings = [3, 2, 1, 0, -1, -2]
#poses_lookaround_turnings = [1, 0, -1]
poses_lookaround = [ # without first joint!
                    [0.523,1.309,-1.919,0], # looking down
                    [0.523,1.047,-1.745,0], # a bit higher
                    [0.523,0.689,-1.745,0], # a bit below horizontally
                    [0.523,0.444,-1.555,0], # a bit above horizontally
                    [0.523,0.444,-1.333,0]  # a bit higher
                    ]
 
for pose in poses_lookaround:
    for turn in poses_lookaround_turnings:
        poses.append([turn]+pose)
        
    poses_lookaround_turnings.reverse()

poses.append(pose_default)

#print poses


""" if the interjacent state is not omitted it needs to have the 'succeeded','aborted','preempted' outcomes """
def get_lookaround_smach(interjacent_state=None):

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')
    
    with sq:
        for i in range(len(poses)):
            sq.add('LOOKAROUND_MOVE_ARM_%d'%i, MoveArm.getMoveArmToJointsSimpleActionState(poses[i]));
            if interjacent_state is not None:
                sq.add('LOOKAROUND_INTERJACENT_STATE_%d'%i, interjacent_state)
    
    return sq

