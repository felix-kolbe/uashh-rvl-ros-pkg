#!/usr/bin/env python

'''
Created on Oct 23, 2013

@author: felix
'''

import roslib; roslib.load_manifest('uashh_smach')

import rospy

from goap.runner import Runner

import goap.config_scitos as config_scitos

import uashh_smach.util as util
from uashh_smach.tasks.tasker import AutonomousGOAPState


def multiple_goals_starter():

    rospy.init_node('multiple_goals_starter')

#    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    runner = Runner(config_scitos)
    sm = util.simple_state_wrapper(AutonomousGOAPState(runner))

    print sm

    util.execute_smach_container(sm, True, 'MULTIPLE_GOALS')

    print 'DONE!'


if __name__ == '__main__':
    multiple_goals_starter()
