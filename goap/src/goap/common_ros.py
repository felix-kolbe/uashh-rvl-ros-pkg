'''
Created on Jul 24, 2013

@author: felix
'''

import roslib; roslib.load_manifest('goap')
import rospy

import rostopic
import actionlib
import tf


from std_msgs.msg import Empty

from move_base_msgs.msg import MoveBaseGoal
import move_base_msgs.msg # for MoveBaseAction, preventing name duplicates
from geometry_msgs.msg import Pose, Point, Quaternion

from common import Action, Condition, Precondition, Effect, VariableEffect


## ROS specific class specializations

class ROSTopicCondition(Condition):

    def __init__(self, state_name, topic, topic_class, field):
        Condition.__init__(self, state_name)
        self._topic = topic
        self._field = field
        self._subscriber = rospy.Subscriber(topic, topic_class, self._callback)
        self._msgeval = rostopic.msgevalgen(field)
        self._value = None

    def __repr__(self):
        return '<%s topic=%s field=%s>' % (self.__class__.__name__, self._topic, self._field)

    def _callback(self, msg):
        self._value = self._msgeval(msg)
#        print 'callback with: ', self._value

    def get_value(self):
        return self._value


## concrete usable classes (no constructor parameters anymore)

class ResetBumperAction(Action):

    def __init__(self):
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), True)],  # TODO: Precondition fails if message wasn't received yet
                            [Effect(Condition.get('robot.bumpered'), False)])
        self._publisher = rospy.Publisher('/bumper_reset', Empty)

    def check_freeform_context(self):
        return self._publisher.get_num_connections() > 0  # unsafe

    def run(self, next_worldstate):
        print 'num of subscribers: ', self._publisher.get_num_connections()
        print 'sending bumper_reset message..'
        self._publisher.publish(Empty())
        rospy.sleep(1)  # TODO: find solution without sleep

# TODO: implement denial of trivial actions (not changing conditions)

class MoveBaseAction(Action):

    class PositionVarEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(self, condition)
        def _is_reachable(self, value):
            return True

    def __init__(self):
        self._condition = Condition.get('robot.pose')
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), False)],
                        [MoveBaseAction.PositionVarEffect(self._condition)])

        self._client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    def check_freeform_context(self):
        # TODO: cache freeform context?
        return self._client.wait_for_server(rospy.Duration(0.1))

    def apply_adhoc_preconditions_for_vareffects(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect.__class__ == MoveBaseAction.PositionVarEffect
        precond_value = start_worldstate.get_condition_value(Condition.get('robot.pose'))
        Precondition(effect._condition, precond_value, None).apply(worldstate)

    def run(self, next_worldstate):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = next_worldstate.get_condition_value(Condition.get('robot.pose'))

        print 'Waiting for base to reach goal...'
        goalstatus = self._client.send_goal_and_wait(goal)
        print 'Goal status: ', goalstatus
