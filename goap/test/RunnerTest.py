#!/usr/bin/env python
'''
Created on Oct 6, 2013

@author: felix
'''
import roslib; roslib.load_manifest('goap')
import rospy

from smach import Sequence

from uashh_smach.util import SleepState, execute_smach_container
from uashh_smach.platform.move_base import WaitForGoalState
from uashh_smach.tasks.tasker import MoveBaseGOAPState

from goap.runner import Runner


def test_runner():
    rospy.init_node('runner_test')

    runner = Runner()

    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                  connector_outcome='succeeded')

    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    with sq:

        Sequence.add('SLEEP', SleepState(5))

        Sequence.add('WAIT_FOR_GOAL', wfg,
                     transitions={'aborted':'SLEEP'})

        Sequence.add('MOVE_BASE_GOAP', MoveBaseGOAPState(runner),
                     transitions={'succeeded':'SLEEP'})

    execute_smach_container(sq, enable_introspection=True)



if __name__ == '__main__':

    test_runner()
