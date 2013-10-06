'''
Created on Sep 9, 2013

@author: felix
'''

import tf

from smach import State

from common import *

from uashh_smach.manipulator.look_around import get_lookaround_smach
from uashh_smach.manipulator.move_arm import get_move_arm_to_joints_positions_state


ARM_FOLDED_POSE = [0, 0.52, 0.52, -1.57, 0]
ARM_FOLDED_POSE_NAMES = ['DH_1_2', 'DH_2_3', 'DH_4_4', 'DH_4_5', 'DH_5_6']
ARM_FOLDED_POSE_NAMED = dict(zip(ARM_FOLDED_POSE_NAMES, ARM_FOLDED_POSE))


class GOAPActionWrapperState(State):
    """Used (by the runner) to add GOAP nodes (aka instances of GOAP actions)
    to a SMACH state machine"""
    def __init__(self, node):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.node = node

    def execute(self, userdata):
        if not self.node.action.check_freeform_context():
            print "Action's freeform context isn't valid! Aborting wrapping state for %s", self.node.action
            return 'aborted'
        next_node = self.node.parent_nodes_path_list[-1]
        self.node.action.run(next_node.worldstate)
        return 'succeeded'


class SmachStateAction(Action):
    """A special Action to wrap a SMACH state.

    Subclass this class to make a SMACH state available to GOAP planning.
    """
    def __init__(self, state, preconditions, effects, **kwargs):
        Action.__init__(self, preconditions, effects, **kwargs)
        self.state = state

    def get_remapping(self):
        """Overload this to set a remapping.
        Actually planned for future use"""
        return {}

    def translate_worldstate_to_userdata(self, next_worldstate, userdata):
        """Overload to make worldstate data available to the state."""
        pass

    def run(self, next_worldstate):
        self.state.execute()
        raise NotImplementedError # yet




class LookAroundAction(SmachStateAction):

    def __init__(self):
        SmachStateAction.__init__(self, get_lookaround_smach(glimpse=True),
                                  [Precondition(Condition.get('arm_can_move'), True)],
                                  [VariableEffect(Condition.get('awareness'))])

    def apply_adhoc_preconditions_for_vareffects(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect is self._effects[0]
        # increase awareness by one
        precond_value = worldstate.get_condition_value(effect._condition) - 1
        Precondition(effect._condition, precond_value, None).apply(worldstate)




class FoldArmAction(SmachStateAction):

    def __init__(self):
        SmachStateAction.__init__(self, get_move_arm_to_joints_positions_state(ARM_FOLDED_POSE),
                                  [Precondition(Condition.get('arm_can_move'), True),
                                   # TODO: why is this anti-effect-precondition needed?
                                   Precondition(Condition.get('robot.arm_folded'), False)],
                                  [Effect(Condition.get('robot.arm_folded'), True)])


