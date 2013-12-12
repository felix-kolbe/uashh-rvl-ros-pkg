
from common import WorldState, Condition
from common import Precondition, Effect, VariableEffect
from common import Goal, Action

from memory import Memory, MemoryCondition

from planning import Node, Planner, PlanExecutor

from runner import Runner


## shutdown check

def is_shutdown():
    return False

def set_shutdown_check(cb):
    import sys
    sys.modules[__name__].is_shutdown = cb

## logging

import logging
_logger = logging.getLogger('rgoap')
"""The logger used in this library"""
_logger.setLevel(logging.INFO)

# add default console output
_loghandler = logging.StreamHandler()
_loghandler.setLevel(logging.INFO)
_loghandler.setFormatter(logging.Formatter('[%(levelname)s] %(message)s'))
_logger.addHandler(_loghandler)

def remove_default_loghandler():
    """Call this to mute this library or to prevent duplicate messages
    when adding another log handler to the logger named 'rgoap'."""
    _logger.removeHandler(_loghandler)
