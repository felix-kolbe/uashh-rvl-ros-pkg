#!/usr/bin/env python

""" This is a task that grabs a box at a predefined position and lays it down one meter away. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf

import math


import smach
import smach_ros
from smach import State, StateMachine, Sequence
from smach_ros import ServiceState, SimpleActionState

#from arm_navigation_msgs.srv import 
#import arm_navigation_msgs
#from arm_navigation_msgs.msg import MoveArmGoal, MoveArmAction, MotionPlanRequest, PositionConstraint, OrientationConstraint, JointConstraint, SimplePoseConstraint

import uashh_smach.manipulator.look_around as look_around      # duplicate import issues
import uashh_smach.manipulator.grab_vertical as grab_vertical

import uashh_smach.platform.move_base as move_base
import uashh_smach.manipulator.move_joints as move_joints
import uashh_smach.manipulator.move_arm as move_arm

import uashh_smach.util as util


BOX_THICKNESS = 0.048
LOOKAROUND_SLEEP_DURATION = 2


def main():
    rospy.init_node('smach')

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')


    sq.userdata.gripper_x = 0.33
    sq.userdata.gripper_y = 0
    sq.userdata.gripper_z = 0.42
    sq.userdata.gripper_phi = math.radians(10)
#    sq.userdata.x = 0.33
#    sq.userdata.y = 0
#    sq.userdata.z = 0.42
#    sq.userdata.phi = 0

    with sq:
        ## Add states to the container
        
        Sequence.add('MOVE_BASE_Forward', move_base.get_move_base_in_odom_state(1, 0))
        
        Sequence.add('MOVE_ARM_GRAB_0',
                       grab_vertical.get_vertical_grab_sequence(-0.23, -0.5, 0.85+0.1, math.radians(90), BOX_THICKNESS, "/base_link")
                       )
        
        
        Sequence.add('MOVE_ARM_ZERO', move_arm.get_move_arm_to_zero_state())
        
        
        Sequence.add('MOVE_BASE_Backward', move_base.get_move_base_in_odom_state(0, 0))
        
        Sequence.add('MOVE_ARM_DROP_0',
                       grab_vertical.get_vertical_drop_sequence(-0.23, -0.5, 0.85+0.1, math.radians(90), BOX_THICKNESS, "/base_link")
                       )
        
        Sequence.add('MOVE_ARM_ZERO_2', move_arm.get_move_arm_to_zero_state())
        
        
        
        
#        Sequence.add('MOVE_ARM_GRAB_0',
#                           grab_vertical.get_vertical_grab_sequence(-0.23, -0.5, 0.85+0.1, math.radians(90), BOX_THICKNESS, "/base_link")
#                           )
#        
#        
#        Sequence.add('MOVE_ARM_ZERO', move_arm.get_move_arm_to_zero_state())
#
#        Sequence.add('MOVE_BASE_Forward', move_base.get_move_base_in_odom_state(1, 0));
#        
#        Sequence.add('MOVE_ARM_DROP_0',
#                           grab_vertical.get_vertical_drop_sequence(-0.23, -0.5, 0.85+0.1, math.radians(90), BOX_THICKNESS, "/base_link")
#                           )
#        
#        Sequence.add('MOVE_ARM_ZERO_2', move_arm.get_move_arm_to_zero_state())
#        
#        
#        Sequence.add('MOVE_BASE_Backward', move_base.get_move_base_in_odom_state(0, 0));
        
        
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sq, '/SM_ROOT')
    sis.start()
    
#    try:
        # Execute the state machine
    outcome = sq.execute()
#    except Exception as ex:
#        print ex

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

