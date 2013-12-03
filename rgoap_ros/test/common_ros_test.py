'''
Created on Jul 25, 2013

@author: felix
'''
import unittest

from rgoap_ros import common_ros

from std_msgs.msg import Bool
import rospy

class TestRTC(unittest.TestCase):

    topic = '/testbool'

    def setUp(self):
        self.rtc = common_ros.ROSTopicCondition('topic.testbool', TestRTC.topic, Bool, '/data')
        rospy.init_node(self.__class__.__name__)

        print self.rtc

    def tearDown(self):
        pass


    def testName(self):
        publisher = rospy.Publisher(TestRTC.topic, Bool, latch=True)

        self.assertIsNone(self.rtc.get_value(), 'New topic cond should have value None.')

        print 'publishing..'

        publisher.publish(True)
        rospy.sleep(1)
        self.assertTrue(self.rtc.get_value(), 'Now topic cond should have value True.')

        publisher.publish(False)
        rospy.sleep(1)
        self.assertFalse(self.rtc.get_value(), 'Now topic cond should have value False.')


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
