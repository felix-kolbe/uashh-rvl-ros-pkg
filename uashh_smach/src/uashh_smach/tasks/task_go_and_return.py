#!/usr/bin/env python

""" This is a task that sends the robot to a position sent from e.g. rviz and returns to the position it stood before moving. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy

from smach import Sequence

import uashh_smach.platform.move_base as move_base
import uashh_smach.util as util



def get_go_and_return_smach():
    sq = Sequence(outcomes=['aborted', 'preempted'],
                  connector_outcome='succeeded')

    sq.userdata.goal_position_x = 1
    sq.userdata.goal_position_y = 0
    sq.userdata.goal_position_yaw = 1
    sq.userdata.saved_position_x = 1
    sq.userdata.saved_position_y = 0
    sq.userdata.saved_position_yaw = 1

    wfg = move_base.WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    with sq:
        ### Add states to the container

        # save position
        Sequence.add('SAVE_ROBOT_POSITION', move_base.ReadRobotPositionState(),
                     remapping={'x':'saved_position_x',
                                'y':'saved_position_y',
                                'yaw':'saved_position_yaw'},
                     )

        # wait for new goal
        Sequence.add('WAIT_FOR_GOAL', wfg,
                     remapping={'x':'goal_position_x',
                                'y':'goal_position_y',
                                'yaw':'goal_position_yaw'}
                     )

        # nav to goal
        Sequence.add('MOVE_BASE_GO', move_base.MoveBaseState('/map'),
                     remapping={'x':'goal_position_x',
                                'y':'goal_position_y',
                                'yaw':'goal_position_yaw'
                                }
                     )

        # wait
        Sequence.add('PAUSE_AT_GOAL', util.SleepState(2))

        # nav back
        Sequence.add('MOVE_BASE_RETURN', move_base.MoveBaseState('/map'),
                     remapping={'x':'saved_position_x',
                                'y':'saved_position_y',
                                'yaw':'saved_position_yaw'
                                },
                     transitions={'succeeded':'SAVE_ROBOT_POSITION'}
                     )

    return sq



def main():
    rospy.init_node('smach')
    util.execute_smach_container(get_go_and_return_smach(), enable_introspection=True)


if __name__ == '__main__':
    main()

