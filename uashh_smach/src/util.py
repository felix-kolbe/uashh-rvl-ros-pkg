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



TAU = math.pi*2   # one tau is one turn. simply as that.




class PromptState(smach.State):
    """Prompt and wait for user action or input on command line.
    
    userdata input prompt: message displayed at prompt 
    userdata output user_input: where the user input is returned
    """ 
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'], input_keys=['prompt'], output_keys=['user_input'])

    def execute(self, userdata):
        rospy.loginfo('Executing PromptState')
        try:
            userdata.user_input = raw_input(userdata.prompt)
            return 'succeeded'
        except EOFError:
            return 'aborted'


class SleepState(smach.State):
    """Sleep for a time duration, given of type rospy Duration or float in seconds at setup time."""
    def __init__(self, duration):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.duration = duration
    
    def execute(self, userdata):
        try:
            rospy.sleep(self.duration)
            return 'succeeded'
        except rospy.ROSInterruptException:        
            return 'aborted'

class SleepStateX(smach.State):
    '''this variant takes the duration via userdata and might be reactivated sometimes.'''
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'], input_keys=['duration'])
    
    def execute(self, userdata):
        try:
            rospy.sleep(userdata.duration)
            return 'succeeded'
        except rospy.ROSInterruptException:        
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
    Those userdata fields have to be passed via 'output_keys'.
    
    If the state outcome should depend on the message content, the msg_cb can dictate the outcome:
    If msg_cb returns True, execute() will return "succeeded".
    If msg_cb returns False, execute() will return "aborted".
    If msg_cb has no return statement, execute() will act as described above.
    
    If thats still not enough, execute() might be overloaded.
    
    latch: If True waitForMsg will return the last received message, so one message might be returned indefinite times.
    timeout: Seconds to wait for a message, defaults to 60.
    output_keys: Userdata keys that the message callback needs to write to. 
    """
    
    def __init__(self, topic, msg_type, msg_cb=None, output_keys=[], latch=False, timeout=60):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],  output_keys=output_keys)
        self.latch = latch
        self.timeout = timeout
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    def waitForMsg(self):
        '''Await and return the message or None on timeout.'''
        rospy.loginfo('Waiting for message...')
        timeout_time = rospy.Time.now() + rospy.Duration.from_sec(self.timeout)
        while rospy.Time.now() < timeout_time:
            self.mutex.acquire()
            if self.msg != None:
                rospy.loginfo('Got message.')
                message = self.msg
                
                if not self.latch:
                    self.msg = None
                
                self.mutex.release()
                return message
            self.mutex.release()
            rospy.sleep(.1)
        
        rospy.loginfo('Timeout on waiting for message!')
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
