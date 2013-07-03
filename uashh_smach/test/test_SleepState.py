#!/usr/bin/env python

import roslib; roslib.load_manifest('uashh_smach')

import rospy

import smach
import smach_ros


from uashh_smach.util import SleepState


def _test_SleepState():
    rospy.init_node('smach')
    
    ud = smach.UserData()
    ud.duration = None
    
    rospy.loginfo("Testing ud:None/arg:1")
    state = SleepState(1)
    outcome = state.execute(ud)
    rospy.loginfo("outcome: %s" % outcome)
    
    rospy.loginfo("Testing ud:2/arg:None")
    ud.duration = 2
    state = SleepState()
    outcome = state.execute(ud)
    rospy.loginfo("outcome: %s" % outcome)

    rospy.loginfo("Testing ud:3/arg:4")
    ud.duration = 3
    state = SleepState(4)
    outcome = state.execute(ud)
    rospy.loginfo("outcome: %s" % outcome)



if __name__ == "__main__":
    _test_SleepState()
