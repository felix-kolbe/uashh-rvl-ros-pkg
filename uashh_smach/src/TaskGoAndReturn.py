#!/usr/bin/env python

""" This is a task that sends the robot to a position sent from e.g. rviz and returns to the position it stood before moving. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf

import math

import smach
import smach_ros
from smach import State, StateMachine, Sequence
from smach_ros import ServiceState, SimpleActionState

import MoveBase
import Util



def main():
    rospy.init_node('smach')

    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')


    sq.userdata.goal_position_x = 1
    sq.userdata.goal_position_y = 0
    sq.userdata.goal_position_yaw = 1
    sq.userdata.saved_position_x = 1
    sq.userdata.saved_position_y = 0
    sq.userdata.saved_position_yaw = 1

    wfg = MoveBase.WaitForGoal() # We don't want multiple subscribers so we need one WaitFor state
    
    with sq:
        '''Add states to the container'''
        
        # save position
        Sequence.add('SAVE_ROBOT_POSITION', MoveBase.ReadRobotPosition(),
                     remapping={'x':'saved_position_x',
                                'y':'saved_position_y',
                                'yaw':'saved_position_yaw'},
                     )
        
        # wait for new goal
        Sequence.add('WAIT_FOR_GOAL', wfg,
#        Sequence.add('WAIT_FOR_GOAL', MoveBase.WaitForGoal(),
                     remapping={'x':'goal_position_x',
                                'y':'goal_position_y',
                                'yaw':'goal_position_yaw'},
#                     transitions={'valid':'MOVE_BASE_GO',
#                                 'invalid':'aborted',
#                                 'preempted':'preempted'}
                         )
        
        # nav to goal
        Sequence.add('MOVE_BASE_GO', MoveBase.MoveBase('/odom'),
                     remapping={'x':'goal_position_x',
                                'y':'goal_position_y',
                                'yaw':'goal_position_yaw'
                                }
                     )
        
        # wait
        Sequence.add('PAUSE_AT_GOAL', Util.SleepState(2))
        
        # nav back
        Sequence.add('MOVE_BASE_RETURN', MoveBase.MoveBase('/odom'),
                     remapping={'x':'saved_position_x',
                                'y':'saved_position_y',
                                'yaw':'saved_position_yaw'
                                },
                     transitions={'succeeded':'SAVE_ROBOT_POSITION'}
                     )
    
    
    
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sq, '/SM_ROOT')
    sis.start()
    
    # Execute the state machine
    outcome = sq.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    rospy.signal_shutdown("shutting down now")
    sis.stop()


if __name__ == '__main__':
    main()

