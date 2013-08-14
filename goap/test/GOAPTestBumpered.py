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
from goap.runner import Runner


class Test(unittest.TestCase):

    def test(self):
        pass



def calc_Pose(x, y, yaw):
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(*quat)
    position = Point(x, y, 0)
    return Pose(position, orientation)



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
#    unittest.main()


    rospy.init_node('goap_bumper_test', log_level=rospy.INFO)

    runner = Runner()

    Condition.add(ROSTopicCondition(
                    'robot.pose', '/odom', Odometry, '/pose/pose'))
    Condition.add(ROSTopicCondition(
                    'robot.bumpered', '/bumper', ScitosG5Bumper, '/motor_stop'))
    Condition.add(MemoryCondition(runner.memory, 'reminded_myself'))


    print 'Waiting to let conditions represent reality...'
    print 'Remember to start topic publishers so conditions make sense instead of None!'
    rospy.sleep(2)
    Condition.initialize_worldstate(runner.worldstate)
    print 'worldstate now is: ', runner.worldstate

    runner.actionbag.add(ResetBumperAction())
    runner.actionbag.add(MoveBaseAction())


    goal = Goal([Precondition(Condition.get('robot.pose'), calc_Pose(1, 0, 0))])

    start_node = runner.update_and_plan(goal, introspection=True)

    print 'start_node: ', start_node

    rospy.sleep(10)

    if start_node is None:
        print 'No plan found! Check you ROS graph!'
    else:
        PlanExecutor().execute(start_node)


    rospy.sleep(20)

