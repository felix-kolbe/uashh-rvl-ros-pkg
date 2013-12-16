'''
Created on Aug 19, 2013

@author: felix
'''
import unittest

import thread
import threading
import random

import rospy
from std_msgs.msg import Int16


class Test(unittest.TestCase):
    def setUp(self):
        pass
    def tearDown(self):
        pass
    def testName(self):
        pass


def myhook():
    rospy.loginfo("shutdown time!")


def myspin():
    rospy.loginfo("spinner running")
    rospy.spin()
    rospy.loginfo("spinner stopped")

def myspinNEW():
    rospy.loginfo("spinnerNEW running")
    rospy.spin()
    rospy.loginfo("spinnerNEW stopped")


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    #unittest.main()

    rospy.init_node('rospy_test')

    rospy.on_shutdown(myhook)

    pub = rospy.Publisher('/int', Int16, latch=True)
    v = random.randint(10, 30)
    rospy.loginfo("publishing: %s", v)
    pub.publish(v)
    rospy.loginfo("published: %s", v)

    thread.start_new_thread(myspin, ())
    rospy.loginfo("spinner started")
    rospy.sleep(1)

    t = threading.Thread(target=myspinNEW)
    t.daemon = True
    t.start()
    rospy.loginfo("thread started")

    rospy.sleep(1)

    rospy.loginfo("sleeping")
    rospy.sleep(5)
    rospy.loginfo("slept")
