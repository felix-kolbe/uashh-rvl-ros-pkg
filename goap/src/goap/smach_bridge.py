'''
Created on Sep 9, 2013

@author: felix
'''

from common import *


from uashh_smach.manipulator.look_around import get_lookaround_smach
from uashh_smach.manipulator.move_arm import get_move_arm_to_joints_positions_state

from uashh_smach.util import execute_smach_container


ARM_FOLDED_POSE = [0, 0.52, 052, -1.57, 0]


class SmachStateAction(Action):

    def __init__(self, state_container, preconditions, effects):
        Action.__init__(self, preconditions, effects)
        self._state_container = state_container


    def run(self, next_worldstate):

        outcome = execute_smach_container(self._state_container,
                                          enable_introspection=True)
        print 'SmachStateAction\'s inner state\'s outcome: ', outcome
        # TODO: handle outcome



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


