'''
Created on Jul 2, 2013

@author: felix
'''
import unittest

import tf

from geometry_msgs.msg import Pose, Point, Quaternion

from rgoap.common import *
from rgoap.inheriting import *
from rgoap.common_ros import *
from rgoap.runner import Runner

import rgoap.config_scitos as config_scitos

from uashh_smach.platform.move_base import position_tuple_to_pose



if __name__ == "__main__":

    rospy.init_node('rgoap_bumper_test', log_level=rospy.INFO)

    runner = Runner(config_scitos)

    Condition.add(MemoryCondition(runner.memory, 'memory.reminded_myself'))

    runner.memory.set_value('awareness', 0)
    runner.memory.set_value('arm_can_move', True)
    runner.memory.set_value('memory.reminded_myself', 333)

    print 'Waiting to let conditions represent reality...'
    print 'Remember to start topic publishers so conditions make sense instead of None!'
    rospy.sleep(2)
    Condition.initialize_worldstate(runner.worldstate)
    print 'worldstate now is: ', runner.worldstate

    runner.actionbag.add(MemoryChangeVarAction(runner.memory, 'memory.reminded_myself', 333, 555))


    goal = Goal([Precondition(Condition.get('robot.pose'), position_tuple_to_pose(1, 0, 0)),
                 Precondition(Condition.get('memory.reminded_myself'), 555)])

    start_node = runner.update_and_plan(goal, introspection=True)

    print 'start_node: ', start_node


    if start_node is None:
        print 'No plan found! Check your ROS graph!'
    else:
        runner.execute_as_smach(start_node, introspection=True)


    rospy.sleep(20)

