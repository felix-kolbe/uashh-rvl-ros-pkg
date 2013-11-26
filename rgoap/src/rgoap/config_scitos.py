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
from smach_bridge import LookAroundAction, FoldArmAction, MoveArmFloorAction




ARM_NAMES = ['DH_1_2', 'DH_2_3', 'DH_4_4', 'DH_4_5', 'DH_5_6']

ARM_POSE_FOLDED = [0, 0.52, 0.52, -1.57, 0]
ARM_POSE_FOLDED_NAMED = dict(zip(ARM_NAMES, ARM_POSE_FOLDED))

ARM_POSE_FLOOR = [0, 0.96, 0.96, -2.0, -1.57]
ARM_POSE_FLOOR_NAMED = dict(zip(ARM_NAMES, ARM_POSE_FLOOR))





def check_joint_msg_matches_pose(msg, pose_dict):
    return all([abs(pose_dict[name] - position) < 0.01
                for (name, position)
                in zip(msg.name, msg.position)
                if name in pose_dict]
               )


def get_all_conditions(memory):
    return [
        # memory
        MemoryCondition(memory, 'arm_can_move', True),
        MemoryCondition(memory, 'awareness', 0),
        # ROS
        ROSTopicCondition('robot.pose', '/odom', Odometry, '/pose/pose'),
        ROSTopicCondition('robot.bumpered', '/bumper', ScitosG5Bumper, '/motor_stop'),
        ROSTopicCondition('robot.arm_folded', '/joint_states', JointState,
                          msgeval=lambda msg: check_joint_msg_matches_pose(msg, ARM_POSE_FOLDED_NAMED)),
        ROSTopicCondition('robot.arm_pose_floor', '/joint_states', JointState,
                          msgeval=lambda msg: check_joint_msg_matches_pose(msg, ARM_POSE_FLOOR_NAMED))
        ]


def get_all_actions(memory):
    return [
        # memory
        # ROS - pure actions
        ResetBumperAction(),
        # ROS - wrapped SMACH states
        MoveBaseAction(),
        LookAroundAction(),
        FoldArmAction(),
        MoveArmFloorAction()
        ]



def get_all_goals(memory):
    return [
        # memory
        # ROS
#        MoveAroundGoal(),
        LocalAwareGoal()
        ]

