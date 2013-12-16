#!/usr/bin/env python

'''
Created on Oct 6, 2013

@author: felix
'''
import roslib; roslib.load_manifest('uashh_smach')

import rospy
import rostopic

from rospy.service import ServiceException

import thread
import threading
import math
from collections import deque

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PoseArray
from task_msgs.msg import TaskActivationAction
from nav_msgs.msg import Path
from hector_nav_msgs.srv import GetRobotTrajectory, GetRobotTrajectoryRequest

from smach import Sequence, StateMachine
from smach_ros import ActionServerWrapper, IntrospectionServer

from rgoap.common import Condition, Goal, Precondition
from rgoap_ros import SMACHRunner
from rgoap_smach import RGOAPRunnerState

from uashh_smach.util import UserDataToOutcomeState, SleepState, get_sleep_until_smach_enabled_smach
from uashh_smach.platform.move_base import WaitForGoalState, get_random_goal_smach, position_tuple_to_pose, calc_random_pose_tuple
from uashh_smach.manipulator.look_around import get_lookaround_smach
from uashh_smach.tasks import task_go_and_return, task_move_around, task_patrol

from uashh_smach import rgoap_subclasses
from uashh_smach.rgoap_subclasses import MoveToPoseGoal




class MoveBaseRGOAPState(RGOAPRunnerState):
    """Use RGOAP to move the robot to a pose that is calculated from x,y,yaw tuple in userdata"""
    def __init__(self, runner):
        RGOAPRunnerState.__init__(self, runner,
                                  input_keys=['x', 'y', 'yaw'],
                                  output_keys=['user_input'])

    def _build_goal(self, userdata):
        pose = position_tuple_to_pose(userdata.x, userdata.y, userdata.yaw)
        return Goal([Precondition(Condition.get('robot.pose'), pose)]) # TODO: prepared goal available: MoveToPoseGoal


class IncreaseAwarenessRGOAPState(RGOAPRunnerState):
    """Use RGOAP to increase the robot's awareness (a memory variable)"""
    def __init__(self, runner):
        RGOAPRunnerState.__init__(self, runner)

    def _build_goal(self, userdata):
        return Goal([Precondition(Condition.get('awareness'), 4)]) # TODO: prepared goal available: LocalAwareGoal



class TaskPosesGoalGenerator(object):

    def __init__(self):
        self.mutex = threading.Lock()
        self._messages = deque()
        self._goal_poses_sub = rospy.Subscriber('/move_base_task/goal', PoseStamped, self._callback)

    def _callback(self, msg):
        self.mutex.acquire()
        self._messages.appendleft(msg)
        self.mutex.release()

    def get_goals(self):
        self.mutex.acquire()
        goals = []

        for i in range(10):
            try:
                # targeted metrics:
                # new message, age 1 seconds: 1 - 1/100 = 0.99
                # old message, age 50 seconds: 1 - 50/100 = 0.5
                msg = self._messages[i]
                age = (rospy.Time.now() - msg.header.stamp).to_sec()
                assert age > 0
                if age < 1000:
                    goals.append(MoveToPoseGoal(msg.pose, msg.header.frame_id,
                                                1 - age / 1000))
            except IndexError:
                break

        self.mutex.release()
        return goals


class RandomGoalGenerator(object):

    def get_goals(self):
        """Uses a simple metric to let the farthest possible goals have a
        usability near 1 and nearer goals a smaller one"""
        distance_max = 2
        random_pose_tuples = [calc_random_pose_tuple(1, distance_max=distance_max)
                              for _ in xrange(10)]

#        # debug usability:
#        for x, y, yaw in random_pose_tuples:
#            distance = math.sqrt(x * x + y * y)
#            usability = distance / distance_max
#            print "distance=%s, usability=%s" % (distance, usability)

        goals = [MoveToPoseGoal(position_tuple_to_pose(x, y, yaw),
                                '/base_link',
                                usability=math.sqrt(x * x + y * y) / distance_max)
                 for x, y, yaw in random_pose_tuples]
        return goals



class HectorExplorationGoalGenerator(object):

    def __init__(self):
        self._service_proxy = rospy.ServiceProxy('/get_exploration_path', GetRobotTrajectory)
        self._planned_paths_pub = rospy.Publisher('/task_planning/goal_paths', Path)

    def get_goals(self):
        """Requests a path from the hector exploration service. For the path's
        final pose a goal with usability 1 is created. For every 5th pose in
        the path a goal is created with decreasing usability.
        """
        request = GetRobotTrajectoryRequest()
        try:
            response = self._service_proxy(request)
        except ServiceException as e:
            rospy.logerr(e)
            return []

        goals = []
        print response
        path = response.trajectory
        self._planned_paths_pub.publish(path)
        poses = path.poses
        usability = 1
        for i in xrange(len(poses) - 1, 0, -5):
            target_pose = poses[i]
            goals.append(MoveToPoseGoal(target_pose.pose,
                                        target_pose.header.frame_id,
                                        usability))
            usability -= 0.01

        return goals


class AutonomousRGOAPState(RGOAPRunnerState):
    """Use RGOAP to achieve autonomous behaviour. Generates various goals to be
    handled by the RGOAP planner"""
    def __init__(self, runner):
        RGOAPRunnerState.__init__(self, runner)
        self._goal_generators = [TaskPosesGoalGenerator(),
                                 RandomGoalGenerator(),
                                 HectorExplorationGoalGenerator()]

        self._static_goals = rgoap_subclasses.get_all_goals(self.runner.memory)
        self._goal_poses_pub = rospy.Publisher('/task_planning/goal_poses', PoseArray, latch=True)

    def _build_goals(self, userdata):
        # static goals
        goals = self._static_goals[:]

        # dynamically generated goals
        for goal_generator in self._goal_generators:
            goals.extend(goal_generator.get_goals())

        # publish all poses (for visualization only)
        pose_array = PoseArray()
        pose_array.header.frame_id = '/map'
        for goal in goals:
            for precondition in goal._preconditions:
                if precondition._condition._state_name == 'robot.pose':
                    pose_array.poses.append(precondition._value)
        self._goal_poses_pub.publish(pose_array)

        return goals


def tasker():

    rospy.init_node('tasker')

    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    runner = SMACHRunner(rgoap_subclasses)


    ## sub machines
    sq_move_to_new_goal = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                   connector_outcome='succeeded')
    with sq_move_to_new_goal:
        Sequence.add('WAIT_FOR_GOAL', wfg)
        Sequence.add('MOVE_BASE_RGOAP', MoveBaseRGOAPState(runner))


    sq_autonomous_rgoap = Sequence(outcomes=['preempted'],
                                  connector_outcome='succeeded')
    with sq_autonomous_rgoap:
        Sequence.add('SLEEP_UNTIL_ENABLED', get_sleep_until_smach_enabled_smach())
        Sequence.add('AUTONOMOUS_RGOAP', AutonomousRGOAPState(runner),
                     transitions={'succeeded':'SLEEP_UNTIL_ENABLED',
                                  'aborted':'SLEEP'})
        Sequence.add('SLEEP', SleepState(5),
                     transitions={'succeeded':'SLEEP_UNTIL_ENABLED'})


    ## tasker machine
    sm_tasker = StateMachine(outcomes=['succeeded', 'aborted', 'preempted',
                                       'field_error', 'undefined_task'],
                             input_keys=['task_goal'])
    with sm_tasker:
        ## add all tasks to be available
        # states using rgoap
        StateMachine.add('MOVE_TO_NEW_GOAL_RGOAP', sq_move_to_new_goal)
        StateMachine.add('INCREASE_AWARENESS_RGOAP', IncreaseAwarenessRGOAPState(runner))
        StateMachine.add('AUTONOMOUS_RGOAP_CYCLE', sq_autonomous_rgoap)
        StateMachine.add('AUTONOMOUS_RGOAP_SINGLE_GOAL', AutonomousRGOAPState(runner))

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
        task_states_labels.sort()  # sort alphabetically and then by _RGOAP
        task_states_labels.sort(key=lambda label: '_RGOAP' in label, reverse=True)

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
