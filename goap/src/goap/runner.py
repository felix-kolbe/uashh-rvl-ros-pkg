'''
Created on Aug 5, 2013

@author: felix
'''

import thread

import rospy
from smach import Sequence, State, StateMachine

import tf

from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion

from uashh_smach.util import WaitForMsgState, CheckSmachEnabledState, TopicToOutcomeState, SleepState, execute_smach_container
from uashh_smach.platform.move_base import WaitForGoalState, get_random_goal_smach
from uashh_smach.manipulator.look_around import get_lookaround_smach

from common import ActionBag, Condition, Goal, Precondition, WorldState
from inheriting import Memory
from planning import Planner, PlanExecutor
from introspection import Introspector

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
            for condition in config_module.get_all_conditions():
                Condition.add(condition)
            for action in config_module.get_all_actions():
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
        # update to reality
        Condition.initialize_worldstate(self.worldstate)

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
        start_node = self.update_and_plan(goal, tries, introspection)
        if start_node is not None:
            PlanExecutor().execute(start_node)



#class MoveBaseGoalListener(object):
#
#    def __init__(self, topic, msg_type):
#        self._subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

class MoveState(State):

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['x', 'y', 'yaw'], output_keys=['user_input'])

        self.runner = Runner(config_scitos)


    def execute(self, ud):
        pose = calc_Pose(ud.x, ud.y, ud.yaw)
        goal = Goal([Precondition(Condition.get('robot.pose'), pose)])
        self.runner.update_and_plan_and_execute(goal, introspection=True)
        return 'succeeded'


def test():
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

        Sequence.add('MOVE_GOAP', MoveState(),
                     transitions={'succeeded':'SLEEP'})


    execute_smach_container(sq, enable_introspection=True)



def test_tasker():

    rospy.init_node('tasker_test')

    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    sq_move_to_new_goal = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                                   connector_outcome='succeeded')
    with sq_move_to_new_goal:
        Sequence.add('WAIT_FOR_GOAL', wfg)
        Sequence.add('MOVE_GOAP', MoveState())


    sm_tasker = StateMachine(outcomes=['field_error'])

    with sm_tasker:
        ## add all tasks to be available
        StateMachine.add('MOVE_TO_NEW_GOAL', sq_move_to_new_goal,
                         transitions={'succeeded':'TASK_RECEIVER',
                                      'aborted':'TASK_RECEIVER',
                                      'preempted':'TASK_RECEIVER'})

        StateMachine.add('LOOK_AROUND', get_lookaround_smach(glimpse=True),
                         transitions={'succeeded':'TASK_RECEIVER',
                                      'aborted':'TASK_RECEIVER',
                                      'preempted':'TASK_RECEIVER'})

        StateMachine.add('MOVE_TO_RANDOM_GOAL', get_random_goal_smach(),
                         transitions={'succeeded':'TASK_RECEIVER',
                                      'aborted':'TASK_RECEIVER',
                                      'preempted':'TASK_RECEIVER'})

        ## now the task receiver is created and automatically links to
        ##   all task states added above
        task_states_labels = sm_tasker.get_children().keys()
        task_receiver_transitions = {'timeout':'TASK_RECEIVER',
                                     'undefined_outcome':'TASK_RECEIVER'}
        task_receiver_transitions.update({l:l for l in task_states_labels})
        StateMachine.add('TASK_RECEIVER',
                         TopicToOutcomeState(task_states_labels, '/task', String, '/data', timeout=200),
                         task_receiver_transitions)

    sm_tasker.set_initial_state(['TASK_RECEIVER'])

    print 'tasker starting, available tasks:', ', '.join(task_states_labels)
    rospy.Publisher('/task/available_tasks', String, latch=True).publish(', '.join(task_states_labels))

    execute_smach_container(sm_tasker, enable_introspection=True)



if __name__ == '__main__':

    #test()
    test_tasker()
