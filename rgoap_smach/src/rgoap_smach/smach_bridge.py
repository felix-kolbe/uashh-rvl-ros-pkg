'''
Created on Sep 9, 2013

@author: felix
'''

from smach import State, StateMachine, UserData

from rgoap import Action



class RGOAPNodeWrapperState(State):
    """Used (by the runner) to add RGOAP nodes (aka instances of RGOAP actions)
    to a SMACH state machine"""
    def __init__(self, node):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.node = node

    def execute(self, userdata):
#        if not self.node.action.is_valid(current_worldstate): TODO: current worldstate not known
#            print "Action isn't valid to worldstate! Aborting executor"
#            print ' action: %r' % self.node.action
#            print ' worldstate:', self.node.worldstate
#            return 'aborted'
#        print "Action %s valid to worldstate" % self.node.action
        if not self.node.action.check_freeform_context():
            print "Action's freeform context isn't valid! Aborting wrapping state for %s", self.node.action
            return 'aborted'
        next_node = self.node.parent_node()
        self.node.action.run(next_node.worldstate)
        return 'succeeded'


class SMACHStateWrapperAction(Action):
    """A special Action to wrap a SMACH state.

    Subclass this class to make a SMACH state available to RGOAP planning.
    """
    def __init__(self, state, preconditions, effects, **kwargs):
        Action.__init__(self, preconditions, effects, **kwargs)
        self.state = state

    def get_remapping(self):
        """Override this to set a remapping.
        Actually planned for future use"""
        return {}

    def translate_worldstate_to_userdata(self, next_worldstate, userdata):
        """Override to make worldstate data available to the state."""
        pass

    def translate_userdata_to_worldstate(self, userdata, next_worldstate):
        # FIXME: translation from userdata does not work
        """Override to make the state's output available to the worldstate."""
        pass

    def run(self, next_worldstate):
        userdata = UserData()
        self.translate_worldstate_to_userdata(next_worldstate, userdata)
        self.state.execute(userdata)
        self.translate_userdata_to_worldstate(userdata, next_worldstate)



class RGOAPRunnerState(State):
    """Subclass this state to activate the RGOAP planner from within a
    surrounding SMACH state container, e.g. the ActionServerWrapper
    """
    # TODO: maybe make this class a smach.Container and add states dynamically?
    def __init__(self, runner, **kwargs):
        State.__init__(self, ['succeeded', 'aborted', 'preempted'], **kwargs)
        self.runner = runner

    def execute(self, userdata):
        try:
            goal = self._build_goal(userdata)
        except NotImplementedError:
            try:
                goals = self._build_goals(userdata)
            except NotImplementedError:
                raise NotImplementedError("Subclass %s neither implements %s nor %s" % (
                                          self.__class__.__name__,
                                          self._build_goal.__name__,
                                          self._build_goals.__name__))
            # else cases are needed to not catch other NotImplementedErrors
            else:
                outcome = self.runner.plan_and_execute_goals(goals)
        else:
            outcome = self.runner.update_and_plan_and_execute(goal, introspection=True)


        print "Generated RGOAP sub state machine returns: %s" % outcome
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return outcome

    def _build_goal(self, userdata):
        """Build and return a rgoap.Goal the planner should accomplish"""
        raise NotImplementedError

    def _build_goals(self, userdata):
        """Build and return a rgoap.Goal list the planner should accomplish"""
        raise NotImplementedError

    def request_preempt(self):
        self.runner.request_preempt()
        State.request_preempt(self)

    def service_preempt(self):
        self.runner.service_preempt()
        State.service_preempt(self)



def rgoap_path_to_smach_container(start_node):
    sm = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    node = start_node
    with sm:
        while not node.is_goal(): # skipping the goal node at the end
            next_node = node.parent_node()

            if isinstance(node.action, SMACHStateWrapperAction):
                # TODO: when smach executes SMACHStateWrapperActions, their action.check_freeform_context() is never called!
                StateMachine.add_auto('%s_%X' % (node.action.__class__.__name__, id(node)),
                                      node.action.state,
                                      ['succeeded'],
                                      remapping=node.action.get_remapping())
                node.action.translate_worldstate_to_userdata(next_node.worldstate, sm.userdata)
            else:
                StateMachine.add_auto('%s_%X' % (node.action.__class__.__name__, id(node)),
                                      RGOAPNodeWrapperState(node),
                                      ['succeeded'])

            node = next_node

    return sm
