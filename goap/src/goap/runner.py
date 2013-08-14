'''
Created on Aug 5, 2013

@author: felix
'''

import rospy
from smach import Sequence, State

import tf

from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

import uashh_smach
from uashh_smach.util import WaitForMsgState, CheckSmachEnabledState, SleepState, execute_smach_container
from uashh_smach.platform.move_base import WaitForGoalState

from geometry_msgs.msg._PoseStamped import PoseStamped


from goap import ActionBag, Condition, Goal, Precondition, WorldState
from inheriting import Memory
from planning import Planner, PlanExecutor

import config_scitos


def calc_Pose(x, y, yaw):
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    orientation = Quaternion(*quat)
    position = Point(x, y, 0)
    return Pose(position, orientation)



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
            for condition in config_module.get_all_conditions():
                Condition.add(condition)
            for action in config_module.get_all_actions():
                self.actionbag.add(action)

        print 'letting conditions receive reality...'
        rospy.sleep(2)

        Condition.initialize_worldstate(self.worldstate)

        self.planner = Planner(self.actionbag, self.worldstate, None)


    def __repr__(self):
        return '<%s memory=%s worldstate=%s actions=%s planner=%s>' % (self.__class__.__name__,
                                self.memory, self.worldstate, self.actionbag, self.planner)

    def update_and_plan(self, goal, tries=1):
        # update to reality
        Condition.initialize_worldstate(self.worldstate)

        while tries > 0:
            tries -= 1
            start_node = self.planner.plan(goal)
            if start_node is not None:
                return start_node

    def update_and_plan_and_execute(self, goal, tries=1):
        start_node = self.update_and_plan(goal, tries)
        if start_node is not None:
            PlanExecutor().execute(start_node)



#class MoveBaseGoalListener(object):
#
#    def __init__(self, topic, msg_type):
#        self._subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

class MoveState(State):

    def __init__(self):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=['x', 'y', 'yaw'], output_keys=['user_input'])

        self.runner = Runner()

    def execute(self, ud):
        pose = calc_Pose(ud.x, ud.y, ud.yaw)
        goal = Goal([Precondition(Condition.get('robot.pose'), pose)])
        self.runner.update_and_plan_and_execute(goal)
        return 'succeeded'


def test():
    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                  connector_outcome='succeeded')

    wfg = WaitForGoalState() # We don't want multiple subscribers so we need one WaitForGoal state

    with sq:

        Sequence.add('SLEEP', SleepState(5))

#        Sequence.add('CHECK', CheckSmachEnabledState(),
#                    transitions={'aborted':'SLEEP'#,
#                                 #'succeeded':'MOVE'
#                                 })

        Sequence.add('WAIT_FOR_GOAL', wfg,
                     transitions={'aborted':'SLEEP'})

        Sequence.add('MOVE', MoveState(),
                     transitions={'succeeded':'SLEEP'})


    execute_smach_container(sq, enable_introspection=True)



if __name__ == '__main__':

    test()
