#!/usr/bin/env python

import roslib; roslib.load_manifest('uashh_smach')

import rospy

import smach
import smach_ros


from util import PromptState, SleepState


def _test_PromptState():
    rospy.init_node('smach')
    
    ud = smach.UserData()
    ud.prompt = "Type anything: "
    ud.user_input = None
    
    state = PromptState()
    outcome = state.execute(ud)
    
    print "PromptState's outcome: ", outcome
    print "userdata's user_input: ", ud.user_input



if __name__ == "__main__":
    _test_PromptState()
