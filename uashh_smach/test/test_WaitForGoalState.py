#!/usr/bin/env python

import roslib; roslib.load_manifest('uashh_smach')

import rospy

import smach
import smach_ros


from uashh_smach.platform.move_base import WaitForGoalState


def _test_WaitForGoalState():
    rospy.init_node('smach')
    wfg = WaitForGoalState()
    print 'execute #1'
    wfg.execute(smach.UserData())
    print 'execute #2'
    wfg.execute(smach.UserData())
    print 'execute #3'
    wfg.execute(smach.UserData())
    #util.execute_smach_container(WaitForGoalState())


if __name__ == "__main__":
    _test_WaitForGoalState()
