'''
Created on Aug 5, 2013

@author: felix
'''
import roslib; roslib.load_manifest('rgoap_ros')
import rospy

import thread

from smach import UserData
from smach_ros import IntrospectionServer

from rgoap import Runner

from rgoap_ros import Introspector
from rgoap_smach import SMACHStateWrapperAction
from rgoap_smach import rgoap_path_to_smach_container


import logging
_logger = logging.getLogger('rgoap.ros')



## from uashh_smach.util import execute_smach_container
def execute_smach_container(smach_container, enable_introspection=False,
                            name='/SM_ROOT', userdata=UserData()):
    if not rospy.core.is_initialized():
        rospy.init_node('smach')

    if enable_introspection:
        # Create and start the introspection server
        sis = IntrospectionServer('smach_executor', smach_container, name)
        sis.start()

    outcome = smach_container.execute(userdata)
    _logger.info("smach outcome: %s", outcome)

    if enable_introspection:
        sis.stop()

    return outcome


class SMACHRunner(Runner):
    """
    This Runner subclass uses SMACH instead of the rgoap.PlanExecutor to
    execute an RGOAP plan.

    If enabled the smach viewer can be used for introspection.
    """

    def __init__(self, *args, **kwargs):
        Runner.__init__(self, *args, **kwargs)

        self._introspector = None
        self._current_smach = None # used to propagate preemption into generated smach


    def _setup_introspection(self):
        # init what could have been initialized externally
        if not rospy.core.is_initialized():
            rospy.init_node('rgoap_runner_introspector')
        # init everything else but only once
        if self._introspector is None:
            self._introspector = Introspector()
            thread.start_new_thread(rospy.spin, ())
            _logger.info("introspection spinner started")
        # TODO: check why spinner does not work [when runner called from unittest?]


    def request_preempt(self):
        Runner.request_preempt(self)
        if self._current_smach is not None:
            self._current_smach.request_preempt()

    def preempt_requested(self):
        return Runner.preempt_requested(self) or (self._current_smach.preempt_requested()
                                                   if self._current_smach is not None
                                                   else False)

    def service_preempt(self):
        Runner.service_preempt(self)
        if self._current_smach is not None:
            self._current_smach.service_preempt()


    def plan(self, goal, introspection=False):
        """plan for given goal and return start_node of plan or None

        introspection: introspect RGOAP planning via smach.introspection
        """
        if introspection:
            self._setup_introspection()

        start_node = Runner.plan(self, goal, introspection)

        if introspection:
            if start_node is not None:
                self._introspector.publish(start_node)
            self._introspector.publish_net(self.planner.last_goal_node, start_node)

        return start_node

    def plan_and_execute_goals(self, goals):
        """Sort goals by usability and try to plan and execute one by one until
        one goal is achieved"""
        self._setup_introspection()
        return Runner.plan_and_execute_goals(self, goals)


    def execute(self, start_node, introspection=False):
        outcome = self.execute_as_smach(start_node, introspection)
        return outcome

    def execute_as_smach(self, start_node, introspection=False):
        sm = rgoap_path_to_smach_container(start_node)
        # TODO: create proxies / userdata info for inner-sm introspection
        self._current_smach = sm
        outcome = execute_smach_container(sm, introspection, name='/RGOAP_GENERATED_SMACH')
        self._current_smach = None
        return outcome
