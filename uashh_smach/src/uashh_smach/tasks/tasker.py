#!/usr/bin/env python

'''
Created on Oct 6, 2013

@author: felix
'''
import roslib; roslib.load_manifest('uashh_smach')

import rospy
import rostopic
import tf

import thread

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from task_msgs.msg import TaskActivationAction


from smach import Sequence, StateMachine
from smach_ros import ActionServerWrapper, IntrospectionServer

from goap.common import Condition, Goal, Precondition
from goap.runner import Runner, GOAPPlannerState

from uashh_smach.util import UserDataToOutcomeState, SleepState
from uashh_smach.platform.move_base import WaitForGoalState, get_random_goal_smach, position_tuple_to_pose, calc_random_pose_tuple
from uashh_smach.manipulator.look_around import get_lookaround_smach
from uashh_smach.tasks import task_go_and_return, task_move_around, task_patrol


import goap.config_scitos as config_scitos




class MoveBaseGOAPState(GOAPPlannerState):
    """Use GOAP to move the robot to a pose that is calculated from x,y,yaw tuple in userdata"""
    def __init__(self, runner):
        GOAPPlannerState.__init__(self, runner,
                                  input_keys=['x', 'y', 'yaw'],
                                  output_keys=['user_input'])

    def build_goal(self, userdata):
        pose = position_tuple_to_pose(userdata.x, userdata.y, userdata.yaw)
        return Goal([Precondition(Condition.get('robot.pose'), pose)])


class IncreaseAwarenessGOAPState(GOAPPlannerState):
    """Use GOAP to increase the robot's awareness (a memory variable)"""
    def __init__(self, runner):
        GOAPPlannerState.__init__(self, runner)

    def build_goal(self, userdata):
        return Goal([Precondition(Condition.get('awareness'), 4)])


def tasker():

    rospy.init_node('tasker')

    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    runner = Runner(config_scitos)


    ## sub machines
    sq_move_to_new_goal = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                   connector_outcome='succeeded')
    with sq_move_to_new_goal:
        Sequence.add('WAIT_FOR_GOAL', wfg)
        Sequence.add('MOVE_BASE_GOAP', MoveBaseGOAPState(runner))


    ## tasker machine
    sm_tasker = StateMachine(outcomes=['succeeded', 'aborted', 'preempted',
                                       'field_error', 'undefined_task'],
                             input_keys=['task_goal'])
    with sm_tasker:
        ## add all tasks to be available
        # states using goap
        StateMachine.add('MOVE_TO_NEW_GOAL_GOAP', sq_move_to_new_goal)
        StateMachine.add('INCREASE_AWARENESS_GOAP', IncreaseAwarenessGOAPState(runner))

        # states from uashh_smach
        StateMachine.add('LOOK_AROUND', get_lookaround_smach())
        StateMachine.add('GLIMPSE_AROUND', get_lookaround_smach(glimpse=True))
        StateMachine.add('MOVE_ARM_CRAZY', get_lookaround_smach(crazy=True))

        StateMachine.add('MOVE_TO_RANDOM_GOAL', get_random_goal_smach())
        StateMachine.add('MOVE_TO_NEW_GOAL_AND_RETURN', task_go_and_return.get_go_and_return_smach())
        StateMachine.add('PATROL_TO_NEW_GOAL', task_patrol.get_patrol_smach())
        StateMachine.add('MOVE_AROUND', task_move_around.get_move_around_smach())

        StateMachine.add('SLEEP_FIVE_SEC', SleepState(5))


        ## now the task receiver is created and automatically links to
        ##   all task states added above
        task_states_labels = sm_tasker.get_children().keys()
        task_states_labels = sorted(task_states_labels)  # sort alphabetically
        task_states_labels = sorted(task_states_labels,  # sort by _GOAP
                                    key=lambda label: '_GOAP' in label,
                                    reverse=True)
        task_receiver_transitions = {'undefined_outcome':'undefined_task'}
        task_receiver_transitions.update({l:l for l in task_states_labels})

        StateMachine.add('TASK_RECEIVER',
                         UserDataToOutcomeState(task_states_labels,
                                                'task_goal',
                                                lambda ud: ud.task_id),
                         task_receiver_transitions)

    sm_tasker.set_initial_state(['TASK_RECEIVER'])

    rospy.loginfo('tasker starting, available tasks: %s', ', '.join(task_states_labels))
    pub = rospy.Publisher('/task/available_tasks', String, latch=True)
    thread.start_new_thread(rostopic.publish_message, (pub, String, [', '.join(task_states_labels)], 1))

    asw = ActionServerWrapper('activate_task', TaskActivationAction,
                              wrapped_container=sm_tasker,
                              succeeded_outcomes=['succeeded'],
                              aborted_outcomes=['aborted', 'undefined_task'],
                              preempted_outcomes=['preempted'],
                              goal_key='task_goal'
                              )

    # Create and start the introspection server
    sis = IntrospectionServer('smach_tasker_action', sm_tasker, '/SM_ROOT')
    sis.start()

    asw.run_server()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':

    tasker()
