#!/usr/bin/env python

'''
Created on Aug 5, 2013

@author: felix
'''
import roslib; roslib.load_manifest('goap')

import thread

import rospy
import rostopic
import tf

from smach import Sequence, State, StateMachine
from smach_ros import ActionServerWrapper, IntrospectionServer

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion
from task_msgs.msg import TaskActivationAction, TaskActivationGoal

from uashh_smach.util import CheckSmachEnabledState, TopicToOutcomeState, UserDataToOutcomeState, SleepState, execute_smach_container
from uashh_smach.platform.move_base import WaitForGoalState, get_random_goal_smach
from uashh_smach.manipulator.look_around import get_lookaround_smach
from uashh_smach.tasks import task_go_and_return, task_move_around, task_patrol

from common import ActionBag, Condition, Goal, Precondition, WorldState
from inheriting import Memory
from planning import Planner, PlanExecutor
from introspection import Introspector
from smach_bridge import SmachStateAction, GOAPActionWrapperState

import config_scitos


def calc_Pose(x, y, yaw):
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(*quat)
    position = Point(x, y, 0)
    return Pose(position, orientation)



class Runner(object):
    """
    self.memory: memory to be used for conditions and actions
    self.worldstate: the default/start worldstate
    self.actionbag: the actions this runner uses
    self.planner: the planner this runner uses
    """

    def __init__(self, config_module=None):
        """
        param:config_module: a scenario/robot specific module to prepare setup,
                that has the following members:
                    get_all_conditions() -> return a list of conditions
                    get_all_actions() -> return a list of actions
        """
        self.memory = Memory()
        self.worldstate = WorldState()
        self.actionbag = ActionBag()

        if config_module is not None:
            for condition in config_module.get_all_conditions(self.memory):
                Condition.add(condition)
            for action in config_module.get_all_actions(self.memory):
                self.actionbag.add(action)

        self.planner = Planner(self.actionbag, self.worldstate, None)

        self._introspector = None


    def __repr__(self):
        return '<%s memory=%s worldstate=%s actions=%s planner=%s>' % (self.__class__.__name__,
                                self.memory, self.worldstate, self.actionbag, self.planner)

    def _setup_introspection(self):
        # init what could have been initialized externally
        if not rospy.core.is_initialized():
            rospy.init_node('goap_runner_introspector')
        # init everything else but only once
        if self._introspector is None:
            self._introspector = Introspector()
            thread.start_new_thread(rospy.spin, ())
            print "introspection spinner started"


    def update_and_plan(self, goal, tries=1, introspection=False):
        """introspection: introspect GOAP planning via smach.introspection"""
        # update to reality
        Condition.initialize_worldstate(self.worldstate)

        print "worldstate initialized/updated to: ", self.worldstate
        for (condition, value) in self.worldstate._condition_values.iteritems():
            if value is None:
                rospy.logwarn("Condition still 'None': %s", condition)

        if introspection:
            self._setup_introspection()

        while tries > 0:
            tries -= 1
            start_node = self.planner.plan(goal=goal)
            if start_node is not None:
                break

        if introspection:
            if start_node is not None:
                self._introspector.publish(start_node)
            self._introspector.publish_net(start_node,
                           self.planner.last_goal_node)

        return start_node


    def update_and_plan_and_execute(self, goal, tries=1, introspection=False):
        """introspection: introspect GOAP planning and SMACH execution via
        smach.introspection"""
        start_node = self.update_and_plan(goal, tries, introspection)
        if start_node is not None:
            #PlanExecutor().execute(start_node)
            return self.execute_as_smach(start_node, introspection)
        else:
            return 'aborted'

    def execute_as_smach(self, start_node, introspection=False):
        sm = self.path_to_smach(start_node)
        outcome = execute_smach_container(sm, introspection, name='/SM_GENERATED')
        return outcome


    def path_to_smach(self, start_node):
        sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        node = start_node
        with sm:
            while len(node.parent_nodes_path_list) > 0: # skipping the goal node at the end
                next_node = node.parent_nodes_path_list[-1]

                if isinstance(node.action, SmachStateAction):
                    StateMachine.add_auto('%s_%X' % (node.action.__class__.__name__, id(node)),
                                          node.action.state,
                                          ['succeeded'],
                                          remapping=node.action.get_remapping())
                    node.action.translate_worldstate_to_userdata(next_node.worldstate, sm.userdata)
                else:
                    StateMachine.add_auto('%s_%X' % (node.action.__class__.__name__, id(node)),
                                          GOAPActionWrapperState(node),
                                          ['succeeded'])

                node = next_node

        return sm

    def print_worldstate_loop(self):
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            # update to reality
            Condition.initialize_worldstate(self.worldstate)
            print self.worldstate
            rate.sleep()





class GOAPPlannerState(State):
    """Subclass this state to activate the GOAP planner from within a
    surrounding state machine, e.g. the ActionServerWrapper"
    """
    def __init__(self, config_module=None, outcomes=['succeeded', 'aborted', 'preempted'], **kwargs):
        State.__init__(self, outcomes=outcomes, **kwargs)
        self.runner = Runner(config_module)

    def execute(self, userdata):
        goal = self.build_goal(userdata)
        # avoid duplicate introspection:
        outcome = self.runner.update_and_plan_and_execute(goal, introspection=False)
        print "Generated GOAP sub state machine returns: %s" % outcome
        return outcome

    def build_goal(self, userdata):
        """Build and return a goap.Goal the planner should accomplish"""
        raise NotImplementedError


class MoveBaseGOAPState(GOAPPlannerState):
    """Use GOAP to move the robot to a pose in userdata"""
    def __init__(self):
        GOAPPlannerState.__init__(self, config_scitos,
                                  input_keys=['x', 'y', 'yaw'],
                                  output_keys=['user_input'])

    def build_goal(self, userdata):
        pose = calc_Pose(userdata.x, userdata.y, userdata.yaw)
        return Goal([Precondition(Condition.get('robot.pose'), pose)])




def test_runner():
    rospy.init_node('runner_test')

    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                  connector_outcome='succeeded')

    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    with sq:

        Sequence.add('SLEEP', SleepState(5))

#        Sequence.add('CHECK', CheckSmachEnabledState(),
#                    transitions={'aborted':'SLEEP'})

        Sequence.add('WAIT_FOR_GOAL', wfg,
                     transitions={'aborted':'SLEEP'})

        Sequence.add('MOVE_BASE_GOAP', MoveBaseGOAPState(),
                     transitions={'succeeded':'SLEEP'})

    execute_smach_container(sq, enable_introspection=True)



def test_tasker():

    rospy.init_node('tasker_test')

    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    sq_move_to_new_goal = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                   connector_outcome='succeeded')
    with sq_move_to_new_goal:
        Sequence.add('WAIT_FOR_GOAL', wfg)
        Sequence.add('MOVE_BASE_GOAP', MoveBaseGOAPState())


    sm_tasker = StateMachine(outcomes=['succeeded', 'aborted', 'preempted',
                                       'field_error', 'undefined_task'],
                             input_keys=['task_goal'])

    with sm_tasker:
        ## add all tasks to be available
        StateMachine.add('MOVE_TO_NEW_GOAL', sq_move_to_new_goal)

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

    #test_runner()
    test_tasker()
