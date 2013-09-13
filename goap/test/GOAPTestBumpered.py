'''
Created on Jul 2, 2013

@author: felix
'''
import unittest

import tf

from geometry_msgs.msg import Pose, Point, Quaternion

from goap.common import *
from goap.inheriting import *
from goap.common_ros import *
from goap.planning import Planner, PlanExecutor
from goap.runner import Runner

import goap.config_scitos as config_scitos



def calc_Pose(x, y, yaw):
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(*quat)
    position = Point(x, y, 0)
    return Pose(position, orientation)



if __name__ == "__main__":

    rospy.init_node('goap_bumper_test', log_level=rospy.INFO)

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


    goal = Goal([Precondition(Condition.get('robot.pose'), calc_Pose(1, 0, 0)),
                 Precondition(Condition.get('memory.reminded_myself'), 555)])

    start_node = runner.update_and_plan(goal, introspection=True)

    print 'start_node: ', start_node

    if start_node is not None:
        runner.execute_as_smach(start_node, introspection=True)


    rospy.sleep(10)

    quit


    if start_node is None:
        print 'No plan found! Check your ROS graph!'
    else:
        PlanExecutor().execute(start_node)


    rospy.sleep(20)

