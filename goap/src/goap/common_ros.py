'''
Created on Jul 24, 2013

@author: felix
'''

import roslib; roslib.load_manifest('goap')
import rospy

from std_msgs.msg import Empty

from goap import Action, Condition, Precondition, Effect


class ResetBumperAction(Action):

    def __init__(self):
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), True)],
                            [Effect(Condition.get('robot.bumpered'), False)])

    def run(self):
        rospy.Publisher('/bumper_reset', Empty).publish()


class ROSTopicCondition(Condition):

    def __init__(self, topic, field, state_name):
        Condition.__init__(self, state_name)
        self._topic = topic
        self._field = field

    def __repr__(self):
        return '<ROSTopicCondition: topic=%s field=%s>' % self._topic, self._field

    def get_value(self, worldstate):
        return NaN
