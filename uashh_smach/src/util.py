#!/usr/bin/env python

""" This file contains general purpose utility states and methods. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf
from std_msgs.msg import Bool

import math


import smach
import smach_ros
#from smach import State, StateMachine, Sequence
#from smach_ros import ServiceState, SimpleActionState



TAU = math.pi*2   # one tau is one turn. simply as that.




class PauseState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'], input_keys=['msg'])

    def execute(self, userdata):
        rospy.loginfo('Executing state PAUSE_STATE')
        raw_input(userdata.msg)
        return 'succeeded'
    

class SleepState(smach.State):
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.duration = duration
    
    def execute(self, userdata):
        try:
            rospy.sleep(self.duration)
            return 'succeeded'
        except rospy.ROSInterruptException:        
            return 'aborted'
        return 'aborted'

'''this variant takes the duration via userdata and might be reactivated sometimes.''' 
class SleepStateX(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'], input_keys=['duration'])
    
    def execute(self, userdata):
        try:
            rospy.sleep(userdata.duration)
            return 'succeeded'
        except rospy.ROSInterruptException:        
            return 'aborted'
        return 'aborted'


'''As it makes no sense to have more than one transform listener, 
here is a global one that has to be initialized via 
init_transform_listener().'''
transform_listener = None

'''Can be called multiple times.'''
def init_transform_listener():
    global transform_listener
    if transform_listener == None:
        transform_listener = tf.TransformListener();



'''Returns a (x,y,yaw) tuple.'''
def get_current_robot_position_in_odom_frame():
    try:
        trans,rot = transform_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(rot)
        return trans[0], trans[1], yaw
    except (tf.LookupException, tf.ConnectivityException) as e:
        print e
        return 0,0,0
    # TODO: i.e. forward exception 



def execute_smach_container(smach_container, enable_introspection=False):
    rospy.init_node('smach')
    
    if enable_introspection:
        # Create and start the introspection server
        sis = smach_ros.IntrospectionServer('server_name', smach_container, '/SM_ROOT')
        sis.start()
        
        outcome = smach_container.execute()
    
        # Wait for ctrl-c to stop the application
        rospy.spin()
        sis.stop()
    else:
        outcome = smach_container.execute()
