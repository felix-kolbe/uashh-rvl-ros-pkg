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

#from arm_navigation_msgs.srv import 
#import arm_navigation_msgs
#from arm_navigation_msgs.msg import MoveArmGoal, MoveArmAction, MotionPlanRequest, PositionConstraint, OrientationConstraint, JointConstraint, SimplePoseConstraint

import look_around      # duplicate import issues
import grab_vertical

import move_base
import move_joints
import move_arm


import util


LOOKAROUND_SLEEP_DURATION = 2


def main():
    rospy.init_node('smach')


    sm = StateMachine(outcomes=['preempted','aborted'])
    #sm.set_initial_state('CHECK_MOVEMENT')
    
    
    with sm:
        StateMachine.add('SLEEP', util.SleepState(1),
                         transitions={'succeeded':'CHECK_ENABLED'})
        StateMachine.add('CHECK_ENABLED', smach_ros.MonitorState("/enable_smach", Bool, monitor_cb),
                         transitions={'invalid':'ARM_LOOK_AROUND',
                                      'valid':'SLEEP',
                                      'preempted':'preempted'})
        StateMachine.add("ARM_LOOK_AROUND", look_around.get_lookaround_smach(util.SleepState(LOOKAROUND_SLEEP_DURATION)),
                         transitions={'succeeded':'CHECK_ENABLED'})
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    
    try:
        # Execute the state machine
        outcome = sm.execute()
    except Exception as ex:
        print ex

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

