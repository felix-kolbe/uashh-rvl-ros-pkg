'''
Created on Jul 5, 2013

@author: felix
'''
import unittest

from rgoap.common import *
from rgoap.inheriting import *
from rgoap.planning import Node
from rgoap.runner import Runner


#@unittest.skip
class TestSimple(unittest.TestCase):

    def setUp(self):
        self.runner = Runner()

        self.memory = self.runner.memory
        self.worldstate = self.runner.worldstate

        self.memory.set_value('memory.counter', 0)

        print self.memory

        Condition._conditions_dict.clear() # start every test without previous conditions

        Condition.add(MemoryCondition(self.memory, 'memory.counter'))

        print Condition.print_dict()

        self.actionbag = self.runner.actionbag
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', 2, 3))
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', 0, 1))
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', 1, 2))
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', -2, 3))

        print self.actionbag

        self.goal = Goal([Precondition(Condition.get('memory.counter'), 3)])

        self.goal_inaccessible = Goal([Precondition(Condition.get('memory.counter'), 4)])

        print self.worldstate

        print self.runner

    def testGoals(self):
        print '==', self.testGoals.__name__
        Condition.initialize_worldstate(self.worldstate) # needed because worldstate was never initialized before
        self.assertFalse(self.goal.is_valid(self.worldstate), 'Goal should not be valid yet')

    def testPlannerPos(self):
        print '==', self.testPlannerPos.__name__
        start_node = self.runner.update_and_plan(self.goal, introspection=True)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 3, 'Start Node should have three actions')
        self.assertEqual(len(start_node.parent_nodes_path_list), 3, 'Start Node should have three parent nodes')

        self.runner.execute_as_smach(start_node, introspection=True)

        import rospy
        rospy.sleep(5) # to latch introspection # TODO: check why spinner does not work [while in unittest]


    def testPlannerNeg(self):
        print '==', self.testPlannerNeg.__name__
        start_node = self.runner.update_and_plan(self.goal_inaccessible)
        print 'start_node found: ', start_node
        self.assertIsNone(start_node, 'There should be no plan')


    def tearDown(self):
        print 'memory was:', self.memory


#@unittest.skip
class TestIncrementer(unittest.TestCase):

    def setUp(self):
        self.runner = Runner()

        self.memory = self.runner.memory
        self.worldstate = self.runner.worldstate

        self.memory.set_value('memory.counter', 0)

        print self.memory

        Condition._conditions_dict.clear() # start every test without previous conditions

        Condition.add(MemoryCondition(self.memory, 'memory.counter'))

        Condition.initialize_worldstate(self.worldstate)

        self.actionbag = self.runner.actionbag
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter'))

        print Condition.print_dict()

        print self.actionbag

        self.goal = Goal([Precondition(Condition.get('memory.counter'), 3)])

        self.goal_inaccessible = Goal([Precondition(Condition.get('memory.counter'), -2)])

        print self.worldstate

        print self.runner

    def testGoals(self):
        print '==', self.testGoals.__name__
        Condition.initialize_worldstate(self.worldstate) # needed because worldstate was never initialized before
        self.assertFalse(self.goal.is_valid(self.worldstate), 'Goal should not be valid yet')

    def testPlannerPos(self):
        print '==', self.testPlannerPos.__name__
        start_node = self.runner.update_and_plan(self.goal)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 3, 'Plan should have three actions')


    def testPlannerPosUnneededCondition(self):
        Condition.add(MemoryCondition(self.memory, 'memory.unneeded'))
        Condition.initialize_worldstate(self.worldstate)
        print 'reinitialized worldstate with unneeded condition: ', self.worldstate
        self.testPlannerPos()


    def testPlannerNeg(self):
        print '==', self.testPlannerNeg.__name__
        start_node = self.runner.update_and_plan(self.goal_inaccessible)
        print 'start_node found: ', start_node
        self.assertIsNone(start_node, 'There should be no plan')

    def testPlannerNegPos(self):
        print '==', self.testPlannerNegPos.__name__
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', -4))
        start_node = self.runner.update_and_plan(self.goal_inaccessible, introspection=True)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 3, 'Plan should have three actions')

        import rospy
        rospy.sleep(5) # to latch introspection # TODO: check why spinner does not work [while in unittest]


    @unittest.skip("deviation does not work yet") # FIXME: deviation does not work yet
    def testPlannerDeviation(self):
        print '==', self.testPlannerDeviation.__name__
        goal_dev = Goal([Precondition(Condition.get('memory.counter'), 2.05, 0.1)])
        start_node = self.runner.update_and_plan(goal_dev)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 2, 'Plan should have two actions')


    def testPlannerBig(self):
        print '==', self.testPlannerBig.__name__
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', -4))
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', 11))
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', 3))
        goal_big = Goal([Precondition(Condition.get('memory.counter'), 23)])
        start_node = self.runner.update_and_plan(goal_big, introspection=True)
        print 'start_node found: ', start_node

        import rospy
        rospy.sleep(5) # to latch introspection # TODO: check why spinner does not work [while in unittest]

        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')


    def testRunnerMultipleGoals(self):
        print '==', self.testRunnerMultipleGoals.__name__
        goal_big = Goal([Precondition(Condition.get('memory.counter'), 23)])
        goals = [self.goal, self.goal_inaccessible, goal_big]
        self.runner.plan_and_execute_goals(goals)


    def tearDown(self):
        print 'memory was:', self.memory



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
