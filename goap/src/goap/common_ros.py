'''
Created on Jul 24, 2013

@author: felix
'''

import roslib; roslib.load_manifest('goap')
import rospy

import rostopic
import tf

from uashh_smach.platform.move_base import MoveBaseState, pose_orientation_to_quaternion

from std_msgs.msg import Empty

from common import Action, Condition, Precondition, Effect, VariableEffect

from smach_bridge import SmachStateAction


## ROS specific class specializations

class ROSTopicCondition(Condition):
    """Mirrors a ROS message field of a topic as its value.
    Note that this Condition's value remains None until a message is received.
    """
    def __init__(self, state_name, topic, topic_class, field=None, msgeval=None):
        Condition.__init__(self, state_name)
        self._topic = topic
        self._field = field
        self._subscriber = rospy.Subscriber(topic, topic_class, self._callback)
        if msgeval is None:
            assert field is not None
            msgeval = rostopic.msgevalgen(field)
        self._msgeval = msgeval

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
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), True)],
                        [Effect(Condition.get('robot.bumpered'), False)])
        self._publisher = rospy.Publisher('/bumper_reset', Empty)

    def check_freeform_context(self):
        return self._publisher.get_num_connections() > 0  # unsafe

    def run(self, next_worldstate):
        print 'num of subscribers: ', self._publisher.get_num_connections()
        print 'sending bumper_reset message..'
        self._publisher.publish(Empty())
        rospy.sleep(1)  # TODO: find solution without sleep
        # TODO: integrate check for asynchronous action bodys

# TODO: implement denial of trivial actions (not changing conditions), if they're actually concerned?

class MoveBaseAction(SmachStateAction):

    def __init__(self):
        self._condition = Condition.get('robot.pose')
        SmachStateAction.__init__(self, MoveBaseState(),
                        [Precondition(Condition.get('robot.bumpered'), False),
                         Precondition(Condition.get('robot.arm_folded'), True)],
                        [VariableEffect(self._condition)])

    def check_freeform_context(self):
        # TODO: cache freeform context?
        return self.state._action_client.wait_for_server(rospy.Duration(0.1))

    def apply_adhoc_preconditions_for_vareffects(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect._condition is self._condition
        precond_value = start_worldstate.get_condition_value(Condition.get('robot.pose'))
        Precondition(effect._condition, precond_value, None).apply(worldstate)

    def translate_worldstate_to_userdata(self, next_worldstate, userdata):
        goal_pose = next_worldstate.get_condition_value(Condition.get('robot.pose'))
        (_roll, _pitch, yaw) = tf.transformations.euler_from_quaternion(
                        pose_orientation_to_quaternion(goal_pose.orientation))
        userdata.x = goal_pose.position.x
        userdata.y = goal_pose.position.y
        userdata.yaw = yaw
