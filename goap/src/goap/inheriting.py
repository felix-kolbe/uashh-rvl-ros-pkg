'''
Created on Jul 25, 2013

@author: felix
'''

from common import *


class Memory(object):
    """Store condition data not representing (or mocking) real world."""

    def __init__(self):
        self._memory = {}

    def __repr__(self):
        return '<Memory %s>' % self._memory

    def declare_state(self, state_name, value=None):
        """Note that the value will not be stored if the state_name already
        exists, even if the current value is None."""
        if state_name not in self._memory:
            self._memory[state_name] = value

    def get_value(self, state_name):
        return self._memory[state_name]

    def set_value(self, state_name, value):
        self._memory[state_name] = value

#    def matches(self, memory):
#        for (k, v) in self._memory.iteritems():
#            if k in memory._memory and memory._memory[k] != v:
#                return False
#        return True


#class MemoryAction(Action):
#
#    def __init(self, memory, preconditions, effects):
#        Action.__init__(self, preconditions, effects)
#        self._memory = memory


class MemorySetVarAction(Action):

    def __init__(self, memory, state_name, new_value, preconditions, effects):
        Action.__init__(self, preconditions, effects)
        self._memory = memory
        self._state_name = state_name
        self._new_value = new_value

        self._memory.declare_state(self._state_name)

    def __repr__(self):
        return '<%s state_name=%s new=%s>' % (self.__class__.__name__, self._state_name, self._new_value)

    def run(self, next_worldstate):
        self._memory.set_value(self._state_name, self._new_value)


class MemoryChangeVarAction(MemorySetVarAction):

    def __init__(self, memory, state_name, old_value, new_value):
        MemorySetVarAction.__init__(self, memory, state_name, new_value,
                [Precondition(Condition.get(state_name), old_value)],
                [Effect(Condition.get(state_name), new_value)]
            )
        self._old_value = old_value

    def __str__(self):
        return '%s:%s=%s->%s' % (self.__class__.__name__,
                    self._state_name, self._old_value, self._new_value)

    def __repr__(self):
        return '<%s state_name=%s old=%s new=%s>' % (self.__class__.__name__,
                    self._state_name, self._old_value, self._new_value)


class MemoryIncrementerAction(Action):

    class IncEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(self, condition)
        def _is_reachable(self, value):
            return True


    def __init__(self, memory, state_name, increment=1):
        self._condition = Condition.get(state_name)
        Action.__init__(self, [], [MemoryIncrementerAction.IncEffect(self._condition)])
        self._memory = memory
        self._state_name = state_name
        self._increment = increment
        self._memory.declare_state(self._state_name)

    def __str__(self):
        return '%s:%s%s=%s' % (self.__class__.__name__,
                               self._state_name,
                               ('-' if self._increment < 0 else '+'),
                               abs(self._increment))

    def __repr__(self):
        return '<%s state_name=%s incr=%s>' % (self.__class__.__name__,
                            self._state_name, self._increment)

    def cost(self):
        return abs(2 - float(1) / abs(self._increment))

    def run(self, next_worldstate):
        self._memory.set_value(self._state_name, self._memory.get_value(self._state_name) + self._increment)

    def apply_adhoc_preconditions_for_vareffects(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect.__class__ == MemoryIncrementerAction.IncEffect
        precond_value = worldstate.get_condition_value(effect._condition) - self._increment
        Precondition(effect._condition, precond_value, None).apply(worldstate)





class MemoryCondition(Condition):

    def __init__(self, memory, state_name):
        Condition.__init__(self, state_name)
        self._memory = memory
        self._state_name = state_name
        memory.declare_state(self._state_name)

    def get_value(self):
        return self._memory.get_value(self._state_name)

