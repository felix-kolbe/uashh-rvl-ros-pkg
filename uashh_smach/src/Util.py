#!/usr/bin/env python

""" This file contains general purpose utility states and methods. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf
from std_msgs.msg import Bool

import math


import smach
import smach_ros
#from smach import State, StateMachine, Sequence
#from smach_ros import ServiceState, SimpleActionState

# define state PAUSE_STATE
class PauseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'], input_keys=['msg'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PAUSE_STATE')
        raw_input(userdata.msg)
        return 'succeeded'
    
#    
#class SleepState(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'], input_keys=['duration'])
#    
#    def execute(self, userdata):
#        try:
#            rospy.sleep(userdata.duration)
#            return 'succeeded'
#        except rospy.ROSInterruptException:        
#            return 'aborted'
#        return 'aborted'

class SleepState(smach.State):
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.duration = duration
    
    def execute(self, userdata):
        try:
            rospy.sleep(self.duration)
            return 'succeeded'
        except rospy.ROSInterruptException:        
            return 'aborted'
        return 'aborted'
