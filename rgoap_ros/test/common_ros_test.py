'''
Created on Jul 25, 2013

@author: felix
'''
import unittest

from rgoap import Condition, Precondition, Effect, Memory, MemoryCondition
from rgoap_ros import common_ros

from std_msgs.msg import Bool
import rospy



class Test(unittest.TestCase):

    topic = '/testbool'

    def setUp(self):
        rospy.init_node(self.__class__.__name__)

        Condition._conditions_dict.clear() # start every test without previous conditions
        memory = Memory()

        self.rtc = common_ros.ROSTopicCondition('topic.testbool', Test.topic, Bool, '/data')
        Condition.add(self.rtc)
        Condition.add(MemoryCondition(memory, 'memory.anyvar'))
        print self.rtc



        self.rta_true = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', True)],
                                             msg_args=[True])
        self.rta_false = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', False)],
                                             msg_args=[False])

        def msg_cb(message, next_worldstate, value):
            message.data = value
            return message
        self.rta_cb_true = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', True)],
                                             msg_cb=lambda msg, ws: msg_cb(msg, ws, True))
        self.rta_cb_false = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', False)],
                                             msg_cb=lambda msg, ws: msg_cb(msg, ws, False))

    def tearDown(self):
        pass


    def testRTC(self):
        publisher = rospy.Publisher(Test.topic, Bool, latch=True)
        rospy.sleep(2) # let subscribers find publishers
        self.assertIsNone(self.rtc.get_value(), 'New topic cond should have value None.')

        print 'publishing..'

        publisher.publish(True)
        rospy.sleep(2)
        self.assertTrue(self.rtc.get_value(), 'Now topic cond should have value True.')

        publisher.publish(False)
        rospy.sleep(2)
        self.assertFalse(self.rtc.get_value(), 'Now topic cond should have value False.')


    def testRTA_msg_args(self):
        print self.rta_true
        print self.rta_false

        rospy.sleep(2) # let subscribers find publishers
        self.assertIsNone(self.rtc.get_value(), 'New topic cond should have value None.')

        print 'publishing via ROSTopicAction..'

        self.rta_true.run(None)
        rospy.sleep(2)
        self.assertTrue(self.rtc.get_value(), 'Now topic cond should have value True.')

        self.rta_false.run(None)
        rospy.sleep(2)
        self.assertFalse(self.rtc.get_value(), 'Now topic cond should have value False.')

    def testRTA_msg_cb(self):
        print self.rta_cb_true
        print self.rta_cb_false

        rospy.sleep(2) # let subscribers find publishers
        self.assertIsNone(self.rtc.get_value(), 'New topic cond should have value None.')

        print 'publishing via ROSTopicAction..'

        self.rta_cb_true.run(None)
        rospy.sleep(2)
        self.assertTrue(self.rtc.get_value(), 'Now topic cond should have value True.')

        self.rta_cb_false.run(None)
        rospy.sleep(2)
        self.assertFalse(self.rtc.get_value(), 'Now topic cond should have value False.')



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
