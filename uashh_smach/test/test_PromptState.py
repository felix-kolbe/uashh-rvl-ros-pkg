#!/usr/bin/env python

import roslib; roslib.load_manifest('uashh_smach')

import rospy

import smach


from uashh_smach.util import PromptState, simple_state_wrapper, execute_smach_container


def _test_PromptState():
    rospy.init_node('smach')

    ud = smach.UserData()
    ud.prompt = "Prompt via userdata: "
    ud.user_input = None

    def exec_state(state):
        sm = simple_state_wrapper(state)
        return execute_smach_container(sm, userdata=ud)

    state = PromptState("Prompt via parameter: ")
    outcome = exec_state(state)

    print "PromptState's outcome: ", outcome
    print "userdata's user_input: ", ud.user_input

    state = PromptState()
    outcome = exec_state(state)

    print "PromptState's outcome: ", outcome
    print "userdata's user_input: ", ud.user_input



if __name__ == "__main__":
    _test_PromptState()
