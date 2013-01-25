#!/usr/bin/env python

""" This is a task to let the robot move around randomly. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf
from std_msgs.msg import Bool

import math


import smach
import smach_ros
from smach import State, StateMachine, Sequence
from smach_ros import ServiceState, SimpleActionState

import look_around

import move_base
import move_arm

import util

from move_base import WaitForMsgStateX

LOOKAROUND_SLEEP_DURATION = 2



class CheckSmachEnabledState(WaitForMsgStateX):
    def __init__(self):
        WaitForMsgStateX.__init__(self, '/enable_smach', Bool, additional_output_keys=[], latch=True) # outcomes=['enabled', 'disabled'], 

    def execute(self, ud):
        msg = WaitForMsgStateX.waitForMsg(self)
        if msg != None and msg.data:
            return 'succeeded'
        else:
            return 'aborted'
        

def main():
    rospy.init_node('smach')


    sm = StateMachine(outcomes=['preempted','aborted'])
    #sm.set_initial_state('CHECK_MOVEMENT')
    
    
    with sm:
        StateMachine.add('SLEEP', util.SleepState(3),
                         transitions={'succeeded':'CHECK_ENABLED'})
        StateMachine.add('CHECK_ENABLED', CheckSmachEnabledState(),
                         transitions={'succeeded':'MOVE_RANDOMLY',
                                      'aborted':'SLEEP'})
        
        StateMachine.add('MOVE_RANDOMLY', move_base.get_random_goal_smach('/base_link'),
                         transitions={'succeeded':'SLEEP',
                                      'aborted':'CHECK_ENABLED'})
        
#        StateMachine.add("ARM_LOOK_AROUND", look_around.get_lookaround_smach(util.SleepState(LOOKAROUND_SLEEP_DURATION)),
#        StateMachine.add('ARM_LOOK_AROUND', util.SleepState(1),    # mockup


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

