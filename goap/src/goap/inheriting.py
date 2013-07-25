'''
Created on Jul 25, 2013

@author: felix
'''

from goap import *



class MemorySetVarAction(Action):

    def __init__(self, variable, new_value, preconditions, effects):
        Action.__init__(self, preconditions, effects)
        self._variable = variable
        self._new_value = new_value
        self._state_name = 'memory.' + self._variable

    def run(self):
        # TODO: check memory vs worldstate access
        Memory().set_value(self._state_name, self._new_value)


class MemoryChangeVarAction(MemorySetVarAction):

    def __init__(self, variable, old_value, new_value):
        """Note that atm. variable is used both for memory variable and for _condition name.""" # TODO: check comment
        MemorySetVarAction.__init__(self, variable, new_value,
                [Precondition(Condition.get('memory.' + variable), old_value)],
                [Effect(Condition.get('memory.' + variable), new_value)]
            )
        self._old_value = old_value

    def __repr__(self):
        return '<MemoryChangeVarAction var=%s old=%s new=%s>' % (self._variable, self._old_value, self._new_value)


class MemoryIncrementerAction(Action):

    class IncEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(condition)
        def _is_reachable(self, value):
            return True # TODO: change reachability from boolean to float

    def __init__(self, variable, increment=1):
        self._condition = Condition.get('memory.' + variable)
        Action.__init__(self, [], [MemoryIncrementerAction.IncEffect(self._condition)])
        self._variable = variable
        self._increment = increment
        Memory().declare_variable(self._condition)

    def __repr__(self):
        return '<MemoryIncrementerAction var=%s incr=%s>' % (self._condition, self._increment)

    def run(self):
        # TODO worldstate needed
        Memory().set_value(self._condition, None) # TODO: value

    def apply_preconditions(self, worldstate):
        # calculate an ad hoc precondition for our variable effect and apply it
        effect_value = self._condition.get_value(worldstate)
        precond_value = self._calc_preconditional_value(worldstate, effect_value)
        Precondition(self._condition, precond_value, None).apply(worldstate)

    def _calc_preconditional_value(self, worldstate, effect_value):
        return effect_value - self._increment



class MemoryCondition(Condition):

    def __init__(self, variable, worldstate):
        Condition.__init__(self, 'memory.' + variable)
        self._variable = variable
        worldstate.memory.declare_variable(self._state_name)

    def __repr__(self):
        return '<MemoryCondition: var=%s>' % self._variable

    def get_value(self, worldstate):
        return worldstate.memory.get_value(self._state_name)

    def set_value(self, worldstate, value):
        worldstate.memory.set_value(self._state_name, value)
