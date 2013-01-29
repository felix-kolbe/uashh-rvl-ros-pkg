#!/usr/bin/env python

""" This file contains general purpose utility states and methods. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf
from std_msgs.msg import Bool

import math
import threading

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

class SleepStateX(smach.State):
    '''this variant takes the duration via userdata and might be reactivated sometimes.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','preempted','aborted'], input_keys=['duration'])
    
    def execute(self, userdata):
        try:
            rospy.sleep(userdata.duration)
            return 'succeeded'
        except rospy.ROSInterruptException:        
            return 'aborted'
        return 'aborted'



class WaitForMsgState(smach.State):
    """This class acts as a generic message listener with blocking, timeout, latch and flexible usage.
    
    It is meant to be extended with a case specific class that initializes this one appropriately 
    and contains the msg_cb (or overloads execute if really needed).
    
    Its waitForMsg method implements the core functionality: waiting for the message, returning 
    the message itself or None on timeout.
    
    Its execute method wraps the waitForMsg and returns succeeded or aborted, depending on the returned 
    message beeing existent or None. Additionally, in the successfull case, the msg_cb, if given, will 
    be called with the message and the userdata, so that a self defined method can convert message data to 
    smach userdata.
    Those userdata fields have to be passed via 'additional_output_keys'.
    
    If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
    If msg_cb returns True, execute() will return "succeeded".
    If msg_cb returns False, execute() will return "aborted".
    If msg_cb has no return statement, execute() will act as described above.
    
    If thats still not enough, execute() might be overloaded.
    
    If latch is True it will return the last received message, so one message might be returned indefinite times.
    """
    
    def __init__(self, topic, msg_type, msg_cb=None, additional_output_keys=[], latch=False):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],  output_keys=additional_output_keys)
        self.latch = latch
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        '''returns the message or None, not an outcome'''
        print 'Waiting for message...'
        # wait for a maximum of .. seconds
        for i in range(0, 30*100):
            self.mutex.acquire()
            if self.msg != None:
                print 'Got message.'
                message = self.msg
                
                if not self.latch:
                    self.msg = None
                
                self.mutex.release()
                return message
            self.mutex.release()
            rospy.sleep(.1)
        
        print 'Timeout!'
        return None

    def execute(self, ud):
        '''Default simplest execute(), see class description.'''
        msg = self.waitForMsg()
        if msg != None:
            # call callback if there is one
            if self.msg_cb != None:
                cb_result = self.msg_cb(msg, ud)
                # check if callback wants to dictate output
                if cb_result != None:
                    if cb_result:
                        return 'succeeded'
                    else:
                        return 'aborted'
            return 'succeeded'
        else:
            return 'aborted'



class CheckSmachEnabledState(WaitForMsgState):
    def __init__(self):
        WaitForMsgState.__init__(self, '/enable_smach', Bool, msg_cb=self._msg_cb, latch=True) # outcomes=['enabled', 'disabled'], 

    def _msg_cb(self, msg, ud):
        return msg != None and msg.data



'''As it makes no sense to have more than one transform listener, 
here is a global one that has to be initialized via init_transform_listener() 
and accessed via get_transform_listener().'''
_transform_listener = None

def get_transform_listener():
    return _transform_listener

def init_transform_listener():
    '''Can safely be called multiple times.'''
    global _transform_listener
    if _transform_listener == None:
        _transform_listener = tf.TransformListener();




def get_current_robot_position_in_odom_frame():
    return get_current_robot_position('/odom')

def get_current_robot_position(frame='/map'):
    '''Returns a (x,y,yaw) tuple. Frame defaults to /map.'''
    try:
        trans,rot = get_transform_listener().lookupTransform(frame, '/base_link', rospy.Time(0))
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
