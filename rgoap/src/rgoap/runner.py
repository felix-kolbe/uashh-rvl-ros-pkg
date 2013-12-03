'''
Created on Aug 5, 2013

@author: felix
'''
from time import sleep

import rgoap

from common import ActionBag, Condition, WorldState, stringify, stringify_dict
from memory import Memory
from planning import Planner, PlanExecutor




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

        self._last_goal = None
        self._preempt_requested = False # preemption mechanism


    def __repr__(self):
        return '<%s memory=%s worldstate=%s actions=%s planner=%s>' % (self.__class__.__name__,
                                self.memory, self.worldstate, self.actionbag, self.planner)


    def request_preempt(self):
        self._preempt_requested = True

    def preempt_requested(self):
        return self._preempt_requested

    def service_preempt(self):
        self._preempt_requested = False


    def _update_worldstate(self):
        """update worldstate to reality"""
        Condition.initialize_worldstate(self.worldstate)
        print "worldstate initialized/updated to: ", self.worldstate

    def update_and_plan(self, goal, tries=1, introspection=False):
        """update worldstate and call self.plan(...)"""
        self._update_worldstate()
        return self.plan(goal, tries, introspection)


    def plan(self, goal, tries=1, introspection=False):
        """plan for given goal and return start_node of plan or None"""
        # check for any still uninitialised condition
        for (condition, value) in self.worldstate._condition_values.iteritems():
            if value is None:
                print "Condition still 'None': %s" % condition #logwarn

        while tries > 0:
            tries -= 1
            # FIXME: retries won't work here if the input does not change
            start_node = self.planner.plan(goal=goal)
            if start_node is not None:
                break

        return start_node


    def plan_and_execute_goals(self, goals):
        """Sort goals by usability and try to plan and execute one by one until
        one goal is achieved"""
        self._update_worldstate()

        # sort goals
        goals.sort(key=lambda goal: goal.usability, reverse=True)

        print "Available goals:"
        print stringify(goals, '\n')

        # plan until plan for one goal found
        for goal in goals:
            # skip goal we used last time
            if self._last_goal is goal:
                continue

            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            plan = self.plan(goal)
            if plan is None:
                continue # try next goal

            # execution
            self._last_goal = goal
            print "Executing most usable goal: ", goal
            print "With plan: ", plan
            outcome = self.execute(plan, introspection=True)
            print "Most usable goal returned: ", outcome
            if outcome == 'aborted':
                continue # try next goal

            return outcome

        print "For no goal a plan could be found!"
        outcome = 'aborted'

        return outcome


    def update_and_plan_and_execute(self, goal, tries=1, introspection=False):
        """loop that updates, plans and executes until the goal is reached"""
        outcome = None
        # replan and retry on failure as long as a plan is found
        while not rgoap.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'

            start_node = self.update_and_plan(goal, tries, introspection)

            if start_node is None:
                # TODO: maybe at this point update and replan? reality might have changed
                print "RGOAP Runner aborts, no plan found!" #logerr
                return 'aborted'

            outcome = self.execute(start_node, introspection)

            if outcome != 'aborted':
                break # retry

            # check failure
            print "RGOAP Runner execution fails, replanning.." #logwarn

            self._update_worldstate()
            if not goal.is_valid(self.worldstate):
                print "Goal isn't valid in current worldstate" #logwarn
            else:
                print "Though goal is valid in current worldstate, the plan execution failed!?" #logerr

        # until we are succeeding or are preempted
        return outcome


    def execute(self, start_node, introspection=False):
        success = PlanExecutor().execute(start_node)
        return success

    def print_worldstate_loop(self):
        while not rgoap.is_shutdown():
            self._update_worldstate()
            print self.worldstate
            sleep(2)

