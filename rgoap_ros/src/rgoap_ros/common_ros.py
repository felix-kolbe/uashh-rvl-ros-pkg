'''
Created on Jul 24, 2013

@author: felix

ROS specific specializations of rgoap classes
'''

import roslib; roslib.load_manifest('rgoap_ros')
import rospy
import rostopic

from rgoap import Condition



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


# TODO: create super class ROSTopicAction(Action):

