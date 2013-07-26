'''
Created on Jul 24, 2013

@author: felix
'''

import roslib; roslib.load_manifest('goap')
import rospy

import rostopic

from std_msgs.msg import Empty

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
        print 'callback with: ', self._value

    def get_value(self):
        return self._value


## concrete usable classes (no constructor parameters anymore)

class ResetBumperAction(Action):

    def __init__(self):
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), True)],
                            [Effect(Condition.get('robot.bumpered'), False)])

    def run(self):
        rospy.Publisher('/bumper_reset', Empty).publish()


class MoveBaseAction(Action):

    class PositionEffect(VariableEffect):
        def __init__(self):
            VariableEffect.__init__(Condition.get('robot.position'))
        def _is_reachable(self, value):
            return True # TODO: change reachability from boolean to float

    def __init__(self):
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), False)],
                        [MoveBaseAction.PositionEffect()])

    def run(self):
#        MoveBaseState() TODO
        pass
