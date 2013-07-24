'''
Created on Jul 10, 2013

@author: felix
'''
import unittest

from goap.goap import Condition


class ConditionTest(unittest.TestCase):


    def setUp(self):
        self.condition1 = Condition()
        self.condition2 = Condition()
        Condition._conditions_dict = {}


    def tearDown(self):
        pass


    def testAdd(self):
        self.assertIs(Condition.add('name1', self.condition1), None, 'Could not add new condition')
        self.assertIs(Condition.add('name2', self.condition2), None, 'Could not add another new condition')

    def testAddSame(self):
        self.assertIs(Condition.add('name1', self.condition1), None, 'Could not add new condition')
#         self.assertRaises(AssertionError, Condition.add, 'name2', self.condition1) #'Could not add another new condition')

    def testGet(self):
        self.assertRaises(AssertionError, Condition.get, 'name_inexistent') # 'Does not fail on getting inexistent condition')

    def testGetSame(self):
        self.assertIs(Condition.add('name', self.condition1), None, 'Could not add new condition')
        self.assertIs(Condition.get('name'), self.condition1, 'Could not get that same condition')



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
