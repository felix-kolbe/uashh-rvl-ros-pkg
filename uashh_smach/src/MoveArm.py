#!/usr/bin/env python

""" This file has helper methods and generates easy to use smach states 
needed to move the robot arm via the arm_navigation's MoveArmAction. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import smach
import smach_ros
from smach import State, StateMachine
from smach_ros import ServiceState, SimpleActionState

#from arm_navigation_msgs.srv import MoveArmGoal, MoveArmAction
import arm_navigation_msgs.msg
from arm_navigation_msgs.msg import MotionPlanRequest, JointConstraint#, MoveArmGoal, MoveArmAction



joint_names = ['DH_1_2',
               'DH_2_3',
               'DH_3_4',
               'DH_4_5',
               'DH_5_6']


""" setup motion plan request
the length of parameter joint_positions must match the length of global variable joint_names
"""
def create_motion_plan_request_for_joints(joint_positions): 
    motion_plan_request = MotionPlanRequest()
    motion_plan_request.group_name = "SchunkArm"
    motion_plan_request.num_planning_attempts = 1
    motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
    motion_plan_request.planner_id = ""
    
    motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(joint_names))]
    for i in range(len(joint_names)):
        motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names[i]
        motion_plan_request.goal_constraints.joint_constraints[i].position = joint_positions[i]
        motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01
        motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01

    return motion_plan_request


def getMoveArmToJointsSimpleActionState(joint_positions):
    arm_goal = arm_navigation_msgs.msg.MoveArmGoal()
    arm_goal.planner_service_name = "ompl_planning/plan_kinematic_path"
    arm_goal.motion_plan_request = create_motion_plan_request_for_joints(joint_positions)
    
    return SimpleActionState('move_SchunkArm',
                                 arm_navigation_msgs.msg.MoveArmAction,
                                 goal=arm_goal
                             )

def getMoveArmToZerosSimpleActionState():
    return getMoveArmToJointsSimpleActionState([0,0,0,0,0])
