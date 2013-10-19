'''
Created on Aug 5, 2013

@author: felix
'''
import roslib; roslib.load_manifest('goap')

import thread

import rospy

from smach import State, StateMachine

from uashh_smach.util import execute_smach_container

from common import ActionBag, Condition, WorldState
from inheriting import Memory
from planning import Planner, PlanExecutor
from introspection import Introspector
from smach_bridge import SmachStateAction, GOAPActionWrapperState



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

    def _update_worldstate(self):
        """update worldstate to reality"""
        Condition.initialize_worldstate(self.worldstate)

    def update_and_plan(self, goal, tries=1, introspection=False):
        """update worldstate and call self.plan(...)"""
        self._update_worldstate()
        return self.plan(goal, tries, introspection)


    def plan(self, goal, tries=1, introspection=False):
        """plan for given goal and return start_node of plan or None

        introspection: introspect GOAP planning via smach.introspection
        """
        print "worldstate initialized/updated to: ", self.worldstate
        # check for any still uninitialised condition
        for (condition, value) in self.worldstate._condition_values.iteritems():
            if value is None:
                rospy.logwarn("Condition still 'None': %s", condition)

        if introspection:
            self._setup_introspection()

        while tries > 0:
            tries -= 1
            # FIXME: retries won't work here if the input does not change
            start_node = self.planner.plan(goal=goal)
            if start_node is not None:
                break

        if introspection:
            if start_node is not None:
                self._introspector.publish(start_node)
            self._introspector.publish_net(self.planner.last_goal_node, start_node)

        return start_node


    def update_and_plan_and_execute(self, goal, tries=1, introspection=False):
        """loop that updates, plans and executes until the goal is reached

        introspection: introspect GOAP planning and SMACH execution via
                smach.introspection, defaults to False
        """
        outcome = None
        # replan and retry on failure as long as a plan is found
        while not rospy.is_shutdown():
            start_node = self.update_and_plan(goal, tries, introspection)

            if start_node is None:
                # TODO: maybe at this point update and replan? reality might have changed
                rospy.logerr("GOAP Runner aborts, no plan found!")
                return 'aborted'

            #success = PlanExecutor().execute(start_node)
            outcome = self.execute_as_smach(start_node, introspection)

            if outcome != 'aborted':
                break; # retry

            # check failure
            rospy.logwarn("GOAP Runner execution fails, replanning..")

            self._update_worldstate()
            if not goal.is_valid(self.worldstate):
                rospy.logwarn("Goal isn't valid in current worldstate")
            else:
                rospy.logerr("Though goal is valid in current worldstate, the plan execution failed!?")

        # until we are succeeding or are preempted
        return outcome

    def execute_as_smach(self, start_node, introspection=False):
        sm = self.path_to_smach(start_node)
        # TODO: create proxies / userdata info for inner-sm introspection
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
            self._update_worldstate()
            print self.worldstate
            rate.sleep()



class GOAPPlannerState(State):
    """Subclass this state to activate the GOAP planner from within a
    surrounding state machine, e.g. the ActionServerWrapper"
    """
    def __init__(self, runner, **kwargs):
        State.__init__(self, ['succeeded', 'aborted', 'preempted'], **kwargs)
        self.runner = runner

    def execute(self, userdata):
        # TODO: propagate preemption request into goap submachine
        # TODO: maybe make this class a smach.Container and add states dynamically?
        goal = self.build_goal(userdata)
        outcome = self.runner.update_and_plan_and_execute(goal, introspection=True)
        print "Generated GOAP sub state machine returns: %s" % outcome
        if self.preempt_requested():
            rospy.logwarn("Preempt request was ignored as GOAPPlannerState cannot"
                          " yet forward it to inner generated machine.")
        return outcome

    def build_goal(self, userdata):
        """Build and return a goap.Goal the planner should accomplish"""
        raise NotImplementedError
