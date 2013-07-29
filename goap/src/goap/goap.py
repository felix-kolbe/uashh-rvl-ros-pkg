'''
Created on Jul 2, 2013

@author: felix
'''




class WorldState(object):
    """Storage for values of conditions."""

    def __init__(self, worldstate=None):
        self._condition_values = {}
        if worldstate is not None:
            self._condition_values.update(worldstate._condition_values)

    def __repr__(self):
        return '<WorldState %X values=%s>' % (id(self), self._condition_values)

    def get_condition_value(self, condition):
        return self._condition_values[condition]

    def set_condition_value(self, condition, value):
        self._condition_values[condition] = value

    def matches(self, start_worldstate):
        """Return whether self is an 'equal subset' of start_worldstate."""
        start_ws_dict = start_worldstate._condition_values
        matches = True
        for (cond, value) in self._condition_values.viewitems():
            if cond in start_ws_dict:
                if not start_ws_dict[cond] == value:
                    matches = False
                    break
        print 'comparing worldstates: ', matches
        print 'mine: ', self._condition_values
        print 'other: ', start_ws_dict
        return matches

#    def apply_effects(self, action): # TODO: replace by direct calls to action.apply_effects()
# delete me       action.apply_effects(self)




## known as state
class Condition(object):
    """The object that makes any kind of robot or system state available."""

    # TODO: maybe convert to singleton
    def __init__(self, state_name):
        self._state_name = state_name

    def get_value(self):
        """Returns the current value, hopefully not blocking."""
        raise NotImplementedError

    def update_value(self, worldstate):
        """Update the condition's current value to the given worldstate."""
        worldstate.set_condition_value(self, self.get_value())



    _conditions_dict = {}

    @classmethod
    def add(cls, name, condition):
        assert name not in cls._conditions_dict, "Condition '" + name + "' had already been added previously!"
        cls._conditions_dict[name] = condition

    @classmethod
    def get(cls, name):
        assert name in cls._conditions_dict, "Condition '" + name + "' has not yet been added!"
        return cls._conditions_dict[name]

    @classmethod
    def print_dict(cls):
        return '<Conditions %s>' % cls._conditions_dict

    @classmethod
    def initialize_worldstate(cls, worldstate):
        """Initialize the given worldstate with all known conditions and their current values."""
        for condition in cls._conditions_dict.values():
            condition.update_value(worldstate)



class Precondition(object):

    def __init__(self, condition, value, deviation=None):
        self._condition = condition
        self._value = value
        self._deviation = deviation

    def __repr__(self):
        return '<Precondition cond=%s value=%s dev=%s>' % (self._condition, self._value, self._deviation)

    def is_valid(self, worldstate):
        cond_value = worldstate.get_condition_value(self._condition)
        if self._deviation is None:
            return cond_value == self._value
        else:
            return abs(cond_value - self._value) <= self._deviation

    def apply(self, worldstate):
        # TODO: deviation gets lost in backwards planner
        worldstate.set_condition_value(self._condition, self._value)



class Effect(object):
    # TODO: integrate conditions beside memory
    # TODO: think about optional deviation

    def __init__(self, condition, new_value):
        self._condition = condition
        self._new_value = new_value

    def apply_to(self, worldstate):
        # TODO: remove me as I'm only for forward planning?
        worldstate.set_condition_value(self._condition, self._new_value)

    def matches_condition(self, worldstate):
        return worldstate.get_condition_value(self._condition) == self._new_value



class VariableEffect(object):

    def __init__(self, condition):
#        Effect.__init__(self, condition, None)
        self._condition = condition

#     def apply_to(self, worldstate):
#         worldstate.memory.set_value(self._condition, self._new_value)

    def matches_condition(self, worldstate):
        return self._is_reachable(worldstate.get_condition_value(self._condition))

    def _is_reachable(self, value):
        raise NotImplementedError


class Goal(object):

    def __init__(self, preconditions):
        self._preconditions = preconditions

    def __repr__(self):
        return '<Goal preconditions=%s>' % self._preconditions

    def is_valid(self, worldstate):
        for precondition in self._preconditions:
            if not precondition.is_valid(worldstate):
                return False
        return True

    def apply_preconditions(self, worldstate):
        for precondition in self._preconditions:
            precondition.apply(worldstate)



class Action(object):

    def __init__(self, preconditions, effects):
        self._preconditions = preconditions
        self._effects = effects

    def __repr__(self):
        return '<Action type=%s>' % self.__class__.__name__

    def run(self):
        raise NotImplementedError

    ## following two for forward planner

    def is_valid(self, worldstate):
        for precondition in self._preconditions:
            if not precondition.is_valid(worldstate):
                return False
        return True

    def apply_effects(self, worldstate):
        for effect in self._effects:
            effect.apply_to(worldstate)

    ## following two for backward planner

    def check_freeform_context(self):
        """Override to add context checks required to run this action that cannot be satisfied by the planner."""
        return True

    def has_satisfying_effects(self, worldstate, unsatisfied_conditions):
        """Return True if at least one of own effects matches unsatisfied_conditions."""
        for effect in self._effects:
            if effect._condition in unsatisfied_conditions: # TODO: maybe put this check into called method
                if effect.matches_condition(worldstate):
                    return True
        return False

    def apply_preconditions(self, worldstate):
        # TODO: make required derivation of variable actions more obvious and fail-safe
        for precondition in self._preconditions:
            precondition.apply(worldstate)



class ActionBag(object):

    def __init__(self):
        self._actions = set()

    def __repr__(self):
        return '<ActionBag %s>' % self._actions

    def add(self, action):
        self._actions.add(action)

    def get(self):
        return self._actions

    def get_sorted(self): # TODO: sort by costs
#         self._actions.sort(cmp=None, key=None, reverse=False)
        return self.get()

    # regressive planning
    def generate_matching_actions(self, start_worldstate, node_worldstate):
        """Generator providing actions that might help between start_worldstate and current node_worldstate."""
        # TODO
        # TODO: This solution does not work when there are actions that produce an empty common_states_set and no valid action is considered - is this actually possible? the start_worldstate should contain every condition ever needed by an action or condition

        # check which conditions differ between start and current node
                    # TODO: rename to common_conditions_set and key to condition
        common_conditions = start_worldstate._condition_values.viewkeys() & node_worldstate._condition_values.viewkeys()
        unsatisfied_conditions = set()
        for condition in common_conditions:
            if start_worldstate._condition_values[condition] != node_worldstate._condition_values[condition]:
                print 'state different: ', condition
                unsatisfied_conditions.add(condition)
            else:
                print 'state equal: ', condition
        # TODO: move that to worldstate

        # check which action might satisfy those conditions
        for action in self._actions:
            if action.has_satisfying_effects(node_worldstate, unsatisfied_conditions):
                print 'helping action: ', action
                yield action
            else:
                print 'helpless action: ', action



