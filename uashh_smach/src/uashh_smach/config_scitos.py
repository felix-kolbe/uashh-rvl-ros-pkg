'''
Created on Nov 26, 2013

@author: felix
'''


ARM_NAMES = ['DH_1_2', 'DH_2_3', 'DH_3_4', 'DH_4_5', 'DH_5_6']

ARM_POSE_FOLDED = [0, 0.52, 0.52, -1.57, 0]
ARM_POSE_FOLDED_NAMED = dict(zip(ARM_NAMES, ARM_POSE_FOLDED))

ARM_POSE_FLOOR = [0, 0.96, 0.96, -2.0, -1.57]
ARM_POSE_FLOOR_NAMED = dict(zip(ARM_NAMES, ARM_POSE_FLOOR))




def check_joint_msg_matches_pose(msg, pose_dict):
    return all(abs(pose_dict[name] - position) < 0.01
               for (name, position)
               in zip(msg.name, msg.position)
               if name in pose_dict
               )

