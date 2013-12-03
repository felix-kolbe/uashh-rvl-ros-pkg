'''
Created on Sep 9, 2013

@author: felix
'''
import unittest
from unittest.case import SkipTest

import time

from smach import Sequence

from rgoap import Condition, Precondition, VariableEffect, Goal
from rgoap import MemoryCondition
from rgoap import Runner

from rgoap_smach import SMACHStateWrapperAction



def get_lookaround_smach_mock():
    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                  connector_outcome='succeeded')
    return sq


class LookAroundAction(SMACHStateWrapperAction):

    def __init__(self):
        SMACHStateWrapperAction.__init__(self, get_lookaround_smach_mock(),
                                  [Precondition(Condition.get('arm_can_move'), True)],
                                  [VariableEffect(Condition.get('awareness'))])

    def _generate_variable_preconditions(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect is self._effects[0]
        # increase awareness by one
        precond_value = worldstate.get_condition_value(effect._condition) - 1
        return [Precondition(effect._condition, precond_value, None)]



class Test(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # rospy.init_node('smach_rgoap_test')
        pass

    def setUp(self):
        self.runner = Runner()

        memory = self.runner.memory
        memory.declare_state('awareness', 0)
        Condition.add(MemoryCondition(memory, 'awareness'))
        memory.declare_state('arm_can_move', True)
        Condition.add(MemoryCondition(memory, 'arm_can_move'))

        self.runner.actionbag.add(LookAroundAction())

        print self.runner.actionbag

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    @SkipTest
    def testName(self):
        goal = Goal([Precondition(Condition.get('awareness'), 2)])

        self.runner.update_and_plan_and_execute(goal, introspection=True)

        time.sleep(15) # to latch introspection # TODO: check why spinner does not work [while in unittest]


    def testStateAction(self):
        Condition.add(MemoryCondition(self.runner.memory, 'robot.pose'))
        Condition.add(MemoryCondition(self.runner.memory, 'robot.bumpered'))



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
