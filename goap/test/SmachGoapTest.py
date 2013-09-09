'''
Created on Sep 9, 2013

@author: felix
'''
import unittest

import rospy

from goap.common import Condition, Precondition, Goal
from goap.runner import Runner
from goap.inheriting import MemoryCondition

from goap.smach_bridge import LookAroundAction

from goap import config_scitos


class Test(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.init_node('smach_goap_test')

    def setUp(self):
        self.runner = Runner() # config_scitos

        memory = self.runner.memory
        memory.declare_state('awareness', 0)
        Condition.add(MemoryCondition(memory, 'awareness'))
        memory.declare_state('arm_can_move', True)
        Condition.add(MemoryCondition(memory, 'arm_can_move'))

        self.runner.actionbag.add(LookAroundAction())

        print self.runner.actionbag

    def tearDown(self):
        pass

    def testName(self):
        goal = Goal([Precondition(Condition.get('awareness'), 2)])

        self.runner.update_and_plan_and_execute(goal, introspection=True)

        rospy.sleep(15) # to latch introspection # TODO: check why spinner does not work [while in unittest]




if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
