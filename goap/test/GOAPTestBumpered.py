'''
Created on Jul 2, 2013

@author: felix
'''
import unittest

import tf

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from metralabs_msgs.msg import ScitosG5Bumper

from goap.goap import *
from goap.inheriting import *
from goap.common_ros import *
from goap.planning import Planner, PlanExecutor


class Test(unittest.TestCase):

    def test(self):
        pass



def calc_Pose(x, y, yaw):
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(*quat)
    position = Point(x, y, 0)
    return Pose(orientation, position)



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
#    unittest.main()


    rospy.init_node('goap_bumper_test')

    memory = Memory()

    Condition.add('robot.pose', ROSTopicCondition(
                    'robot.pose', '/odom', Odometry, '/pose/pose'))
    Condition.add('robot.bumpered', ROSTopicCondition(
                    'robot.bumpered', '/bumper_state', ScitosG5Bumper, '/bumper_pressed'))
    Condition.add('memory.reminded_myself', MemoryCondition(memory, 'reminded_myself'))

    worldstate = WorldState()

    print 'Waiting to let conditions represent reality...'
    rospy.sleep(2)
    Condition.initialize_worldstate(worldstate)

    actionbag = ActionBag()
    actionbag.add(ResetBumperAction())
    actionbag.add(MoveBaseAction())


    goal = Goal([Precondition(Condition.get('robot.pose'), calc_Pose(3, 2, 1))])

    planner = Planner(actionbag, worldstate, goal)

    start_node = planner.plan()

    print 'start_node: ', start_node

    PlanExecutor().execute(start_node)


    ## init / spin to let conditions know reality




