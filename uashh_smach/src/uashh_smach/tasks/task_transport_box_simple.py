#!/usr/bin/env python

""" This is a task that grabs a box at a predefined position and lays it down one meter away. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy

import math

from smach import Sequence

import uashh_smach.manipulator.grab_vertical as grab_vertical
import uashh_smach.platform.move_base as move_base
import uashh_smach.manipulator.move_arm as move_arm

import uashh_smach.util as util


BOX_THICKNESS = 0.048
LOOKAROUND_SLEEP_DURATION = 2



def _get_transport_box_smach():
    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                  connector_outcome='succeeded')

    sq.userdata.gripper_x = 0.33
    sq.userdata.gripper_y = 0
    sq.userdata.gripper_z = 0.42
    sq.userdata.gripper_phi = math.radians(10)
#    sq.userdata.x = 0.33
#    sq.userdata.y = 0
#    sq.userdata.z = 0.42
#    sq.userdata.phi = 0

    with sq:
        ## Add states to the container

        Sequence.add('MOVE_BASE_Forward', move_base.get_move_base_in_odom_state(1, 0))

        Sequence.add('MOVE_ARM_GRAB_0',
                     grab_vertical.get_vertical_grab_sequence(-0.23, -0.5, 0.85 + 0.1,
                                                              math.radians(90),
                                                              BOX_THICKNESS,
                                                              "/base_link")
                     )


        Sequence.add('MOVE_ARM_ZERO', move_arm.get_move_arm_to_zero_state())


        Sequence.add('MOVE_BASE_Backward', move_base.get_move_base_in_odom_state(0, 0))

        Sequence.add('MOVE_ARM_DROP_0',
                     grab_vertical.get_vertical_drop_sequence(-0.23, -0.5, 0.85 + 0.1,
                                                              math.radians(90),
                                                              BOX_THICKNESS,
                                                              "/base_link")
                     )

        Sequence.add('MOVE_ARM_ZERO_2', move_arm.get_move_arm_to_zero_state())




#        Sequence.add('MOVE_ARM_GRAB_0',
#                     grab_vertical.get_vertical_grab_sequence(-0.23, -0.5, 0.85+0.1,
#                                                              math.radians(90),
#                                                              BOX_THICKNESS,
#                                                              "/base_link")
#                     )
#
#
#        Sequence.add('MOVE_ARM_ZERO', move_arm.get_move_arm_to_zero_state())
#
#        Sequence.add('MOVE_BASE_Forward', move_base.get_move_base_in_odom_state(1, 0));
#
#        Sequence.add('MOVE_ARM_DROP_0',
#                     grab_vertical.get_vertical_drop_sequence(-0.23, -0.5, 0.85+0.1,
#                                                              math.radians(90),
#                                                              BOX_THICKNESS,
#                                                              "/base_link")
#                     )
#
#        Sequence.add('MOVE_ARM_ZERO_2', move_arm.get_move_arm_to_zero_state())
#
#
#        Sequence.add('MOVE_BASE_Backward', move_base.get_move_base_in_odom_state(0, 0));

    return sq


def main():
    rospy.init_node('smach')
    util.execute_smach_container(_get_transport_box_smach(), enable_introspection=True)


if __name__ == '__main__':
    main()

