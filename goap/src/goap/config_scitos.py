'''
Created on Aug 5, 2013

@author: felix
'''

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from metralabs_msgs.msg import ScitosG5Bumper


from common_ros import MoveBaseAction, ResetBumperAction, ROSTopicCondition
from common_ros import LocalAwareGoal
from inheriting import MemoryCondition
from smach_bridge import LookAroundAction, FoldArmAction, ARM_FOLDED_POSE_NAMED


def get_all_conditions(memory):
    return [
        # memory
        MemoryCondition(memory, 'arm_can_move', True),
        MemoryCondition(memory, 'awareness', 0),
        # ROS
        ROSTopicCondition('robot.pose', '/odom', Odometry, '/pose/pose'),
        ROSTopicCondition('robot.bumpered', '/bumper', ScitosG5Bumper, '/motor_stop'),
        ROSTopicCondition('robot.arm_folded', '/joint_states', JointState,
                          msgeval=lambda msg: all([abs(ARM_FOLDED_POSE_NAMED[name] -
                                                       position) < 0.01
                                                   for (name, position)
                                                   in zip(msg.name, msg.position)
                                                   if name in ARM_FOLDED_POSE_NAMED]
                                                  ))
        ]


def get_all_actions(memory):
    return [
        # memory
        # ROS - pure actions
        ResetBumperAction(),
        # ROS - wrapped SMACH states
        MoveBaseAction(),
        LookAroundAction(),
        FoldArmAction()
        ]



def get_all_goals(memory):
    return [
        # memory
        # ROS
#        MoveAroundGoal(),
        LocalAwareGoal()
        ]

