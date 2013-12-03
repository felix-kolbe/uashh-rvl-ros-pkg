#!/usr/bin/env python

""" This file has helper methods and generates easy to use smach states
needed to move the robot arm via the arm_navigation's MoveArmAction. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy

from smach_ros import SimpleActionState

import arm_navigation_msgs.msg  # for MoveArmGoal, MoveArmAction
from arm_navigation_msgs.msg import MotionPlanRequest, JointConstraint

from uashh_smach.config_scitos import ARM_NAMES as JOINT_NAMES



def create_motion_plan_request_for_joints(joint_positions):
    """Setup motion plan request.
    The length of parameter joint_positions must match the length of global variable JOINT_NAMES.
    """
    motion_plan_request = MotionPlanRequest()
    motion_plan_request.group_name = "SchunkArm"
    motion_plan_request.num_planning_attempts = 1
    motion_plan_request.allowed_planning_time = rospy.Duration(5.0)
    motion_plan_request.planner_id = ""

    motion_plan_request.goal_constraints.joint_constraints = [JointConstraint() for i in range(len(JOINT_NAMES))]
    for i in range(len(JOINT_NAMES)):
        motion_plan_request.goal_constraints.joint_constraints[i].joint_name = JOINT_NAMES[i]
        motion_plan_request.goal_constraints.joint_constraints[i].position = joint_positions[i]
        motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01
        motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01

    return motion_plan_request


def get_move_arm_to_joints_positions_state(joint_positions):
    arm_goal = arm_navigation_msgs.msg.MoveArmGoal()
    arm_goal.planner_service_name = "ompl_planning/plan_kinematic_path"
    arm_goal.motion_plan_request = create_motion_plan_request_for_joints(joint_positions)

    return SimpleActionState('move_SchunkArm',
                                 arm_navigation_msgs.msg.MoveArmAction,
                                 goal=arm_goal
                             )

def get_move_arm_to_zero_state():
    return get_move_arm_to_joints_positions_state([0, 0, 0, 0, 0])
