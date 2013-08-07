'''
Created on Aug 5, 2013

@author: felix
'''

from common_ros import MoveBaseAction, ResetBumperAction, ROSTopicCondition


from nav_msgs.msg import Odometry
from metralabs_msgs.msg import ScitosG5Bumper


def get_all_conditions():
    return [
        # memory
        # ROS
        ROSTopicCondition('robot.pose', '/odom', Odometry, '/pose/pose'),
        ROSTopicCondition('robot.bumpered', '/bumper', ScitosG5Bumper, '/motor_stop')
        ]


def get_all_actions():
    return [
        # memory
        # ROS
        MoveBaseAction(),
        ResetBumperAction()
        ]

