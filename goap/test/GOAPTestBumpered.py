'''
Created on Jul 2, 2013

@author: felix
'''
import unittest

from goap.goap import *
from goap.inheriting import *
from goap.common_ros import *


class Test(unittest.TestCase):

    def test(self):
        pass




if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()


    actionbag = ActionBag()
    actionbag.add(ResetBumperAction())

    Condition.add('robot.bumpered', ROSTopicCondition('/bumper_state', 'bumper_locked'))


    Condition.add('memory.reminded_myself', MemoryCondition('reminded_myself'))



    goal = Goal(Condition.get('robot.bumpered'), False)



    ## init / spin to let conditions know reality




