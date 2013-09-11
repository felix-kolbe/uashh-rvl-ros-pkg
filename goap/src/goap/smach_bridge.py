'''
Created on Sep 9, 2013

@author: felix
'''

import tf

from smach import State

from common import *

from uashh_smach.manipulator.look_around import get_lookaround_smach
from uashh_smach.manipulator.move_arm import get_move_arm_to_joints_positions_state


ARM_FOLDED_POSE = [0, 0.52, 052, -1.57, 0]


class GOAPActionWrapperState(State):
    """Used (by the planner) to add GOAP actions to a SMACH state machine"""
    def __init__(self, action, next_worldstate):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.action = action
        self._next_worldstate = next_worldstate

    def execute(self, userdata):
        if not self.action.check_freeform_context():
            print 'Action\'s freeform context isn\'t valid! Aborting wrapping state for %s', self.action
            return 'aborted'
        self.action.run(self._next_worldstate)
        return 'succeeded'


class SmachStateAction(Action):

    def __init__(self, state, preconditions, effects, **kwargs):
        Action.__init__(self, preconditions, effects, **kwargs)
        self.state = state

    def get_remapping(self):
        """Overload this to set a remapping.
        Actually planned for future use"""
        return {}

    def translate_worldstate_to_userdata(self, next_worldstate, userdata):
        raise NotImplementedError




class LookAroundAction(SmachStateAction):

    class IncEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(self, condition)
        def _is_reachable(self, value):
            return True


    def __init__(self):
        SmachStateAction.__init__(self, get_lookaround_smach(glimpse=True),
                                  [Precondition(Condition.get('arm_can_move'), True)],
                                  [LookAroundAction.IncEffect(Condition.get('awareness'))])

    def apply_adhoc_preconditions_for_vareffects(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect.__class__ == LookAroundAction.IncEffect
        precond_value = worldstate.get_condition_value(effect._condition) - 1
        Precondition(effect._condition, precond_value, None).apply(worldstate)




class FoldArm(SmachStateAction):

    def __init__(self):
        SmachStateAction.__init__(self, get_move_arm_to_joints_positions_state(ARM_FOLDED_POSE)
                                  [Precondition(Condition.get('arm_can_move'), True)],
                                  [LookAroundAction.Effect('arm_folded', True)])


