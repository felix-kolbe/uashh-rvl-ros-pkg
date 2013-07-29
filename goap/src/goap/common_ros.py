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

from goap import Action, Condition, Precondition, Effect, VariableEffect


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
        return '<ROSTopicCondition topic=%s field=%s>' % (self._topic, self._field)

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

    def run(self, next_worldstate):
        rospy.Publisher('/bumper_reset', Empty).publish()

# TODO: implement denial of trivial actions (not changing conditions)

class MoveBaseAction(Action):

    class PositionEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(self, condition)
        def _is_reachable(self, value):
            return True # TODO: change reachability from boolean to float

    def __init__(self):
        self._condition = Condition.get('robot.pose')
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), False)],
                        [MoveBaseAction.PositionEffect(self._condition)])

        self._client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    def check_freeform_context(self):
        # TODO: cache freeform context?
        return not self._client.wait_for_server(rospy.Duration(0.1))

    def apply_preconditions(self, worldstate): # TODO: move this to new class VariableAction?
        Action.apply_preconditions(self, worldstate) # apply fix preconditions
        # calculate an ad hoc precondition for our variable effect and apply it
        effect_value = worldstate.get_condition_value(self._condition)
        precond_value = self._calc_preconditional_value(worldstate, effect_value)
        Precondition(self._condition, precond_value, None).apply(worldstate)

    def _calc_preconditional_value(self, worldstate, effect_value):
        start_pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)) # TODO: need to access the start worldstate here
        return start_pose

    def run(self, next_worldstate):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = next_worldstate.get_condition_value(Condition.get('robot.pose'))

        print 'Waiting for base to reach goal...'
        self._client.send_goal_and_wait(goal)
