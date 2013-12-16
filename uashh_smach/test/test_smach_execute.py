#!/usr/bin/env python

import roslib; roslib.load_manifest('uashh_smach')

import rospy

import smach
import smach_ros

from smach import Sequence

from uashh_smach.util import SleepState
from uashh_smach.util import CheckSmachEnabledState
from uashh_smach.util import TransformListenerSingleton

def _test_smach_execute():
    rospy.init_node('smach')
    ud = smach.UserData()
    
#    state = SleepState(12)
    state = CheckSmachEnabledState()
    
    rospy.loginfo("Executing test...")   
    outcome = state.execute(ud)
    
    rospy.loginfo("outcome: %s" % outcome)


def myhook():
    print "shutdown time!"


def _test_smach_execute2():
    rospy.init_node('smach')
    
    rospy.on_shutdown(myhook)
    
    sq = Sequence(
        outcomes = ['succeeded','aborted','preempted'],
        connector_outcome = 'succeeded')
    
    with sq:
        Sequence.add('CHECK', CheckSmachEnabledState(),
                     transitions={'aborted':'SLEEP'})
        Sequence.add('SLEEP', SleepState(10))
    
    rospy.loginfo("Executing test...")
    TransformListenerSingleton.get()
    #outcome = sq.execute()
    
    #rospy.loginfo("outcome: %s" % outcome)
       
    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sq, '/SM_ROOT')
    sis.start()
    
    
    # Execute the state machine
    print "EXECUTING.."
    outcome = sq.execute()

    # Wait for ctrl-c to stop the application
    print "SPINNING.ms."
    rospy.spin()
    #rospy.signal_shutdown("shutting down now")
    sis.stop()
    



if __name__ == "__main__":
    _test_smach_execute2()
