#!/usr/bin/env python

'''
Created on Oct 23, 2013

@author: felix
'''

import roslib; roslib.load_manifest('uashh_smach')

import rospy

from rgoap.runner import Runner

import uashh_smach.util as util
import uashh_smach.rgoap_subclasses as rgoap_subclasses
from uashh_smach.tasks.tasker import AutonomousRGOAPState


def multiple_goals_starter():

    rospy.init_node('multiple_goals_starter')

#    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    runner = Runner(rgoap_subclasses)
    sm = util.simple_state_wrapper(AutonomousRGOAPState(runner))

    print sm

    util.execute_smach_container(sm, True, '/MULTIPLE_GOALS')

    print 'DONE!'


if __name__ == '__main__':
    multiple_goals_starter()
