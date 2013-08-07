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

    def __init__(self):
        self.memory = Memory()
        self.worldstate = WorldState()
        self.actionbag = ActionBag()

        for condition in config_scitos.get_all_conditions():
            Condition.add(condition)

        for action in config_scitos.get_all_actions():
            self.actionbag.add(action)

        rospy.sleep(2) # let conditions receive reality

        Condition.initialize_worldstate(self.worldstate)

        self.planner = Planner(self.actionbag, self.worldstate, None)

    def start(self, goal):
        self.current_goal = goal

        Condition.initialize_worldstate(self.worldstate)

        start_node = self.planner.plan(self.current_goal)

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
        self.runner.start(goal)
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
                     transitions={'aborted':'SLEEP'}
                     )

        Sequence.add('MOVE', MoveState(),
                     transitions={'succeeded':'SLEEP'})


    execute_smach_container(sq, enable_introspection=True)



if __name__ == '__main__':

    test()
