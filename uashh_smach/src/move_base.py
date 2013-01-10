#!/usr/bin/env python

""" This file generates easy to use smach states needed to move the robot base. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf

import math
import random
import threading

import smach
import smach_ros
from smach import State
from smach_ros import ServiceState, SimpleActionState

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion 

import util



def pose_orientation_to_quaternion(msg):
    return [msg.x, msg.y, msg.z, msg.w]



    
def get_move_base_in_map_state(x, y):
    return get_move_base_state("/map", x, y)

def get_move_base_in_odom_state(x, y):
    return get_move_base_state("/odom", x, y)


def get_move_base_random_state():
    radius = random.random()*2 + 1  # 1-3 m
    #yaw = random.random()*TAU/2 - TAU/4    # +-90 deg
    yaw = random.random()*util.TAU*3/4 - util.TAU*3/8    # +-135 deg
    
    return get_move_base_state("/base_link", math.cos(yaw)*radius, math.sin(yaw)*radius, yaw)
    
'''Returns a MoveBaseGoal state which goal parameters are given via parameters at setup time.'''
def get_move_base_state(frame, x=0, y=0, yaw=0):
    base_goal = MoveBaseGoal()
    base_goal.target_pose.header.frame_id = frame
    base_goal.target_pose.header.stamp = rospy.Time.now()
        
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    base_goal.target_pose.pose.orientation = Quaternion(*quat)
    base_goal.target_pose.pose.position = Point(x, y, 0)
    
    return SimpleActionState('move_base', 
                             MoveBaseAction, 
                             goal=base_goal
                             )



'''move_base state with userdata input
frame defaults to '/odom' if not given'''
class MoveBaseState(SimpleActionState):
    def __init__(self, frame='/map'):
        SimpleActionState.__init__(self, 'move_base', MoveBaseAction, input_keys=['x', 'y', 'yaw'], goal_cb=self._goal_cb)
        self.frame = frame
    
    def _goal_cb(self, userdata, old_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame
        goal.target_pose.header.stamp = rospy.Time.now()

        quat = tf.transformations.quaternion_from_euler(0, 0, userdata.yaw)
        goal.target_pose.pose.orientation = Quaternion(*quat)
        goal.target_pose.pose.position = Point(userdata.x, userdata.y, 0)
        return goal

        
    


'''This class acts as an generic message listener with blocking and timeout.
It is meant to be extended with a case specific class that initializes this one appropriately,
 amongst others with a message callback that is called from this class' execute() with the recieved message,
 but not after timeout.'''  
class WaitForMsgState(smach.State):
    def __init__(self, topic, msg_type, msg_cb, additional_output_keys=[]):
        print '_init'
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],  output_keys=additional_output_keys)
        self.mutex = threading.Lock()
        self.msg = None
        self.msg_cb = msg_cb
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback, queue_size=1)

    def _callback(self, msg):
        print '_callback'
        self.mutex.acquire()
#        print '_callback: msg was: '+str(self.msg)
        self.msg = msg
        self.mutex.release()

    def execute(self, ud):
        print 'Waiting for message...'
        # wait for a maximum of .. seconds
        for i in range(0, 30*100):
#            print 'gg_testing'
            self.mutex.acquire()
#            print 'msg currently is: '+str(self.msg)
            if self.msg != None:
                print 'Got message: '+str(self.msg)
                self.msg_cb(self.msg, ud)
                self.msg = None
#                print 'setting to None, now is: '+str(self.msg)
                
                self.mutex.release()
                return 'succeeded'
            self.mutex.release()
            rospy.sleep(1)
        
        print 'Timeout!'
        return 'aborted'


class WaitForGoalState(WaitForMsgState):
    def __init__(self):
        WaitForMsgState.__init__(self, '/move_base_task/goal', PoseStamped, self._msg_cb, additional_output_keys=['x', 'y', 'yaw'])

    def _msg_cb(self, msg, ud):
        ud.x = msg.pose.position.x
        ud.y = msg.pose.position.y
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(pose_orientation_to_quaternion(msg.pose.orientation))
        ud.yaw = yaw
        
        

    
'''This class acts as an generic message listener with blocking and timeout.
It is meant to be extended with a case specific class that initializes this one appropriately
 and calls this class' waitForMsg() and handles its returned message as needed from within its own execute(). 
 That execute() will be called by smach and has to return 'succeeded' or 'aborted' as an outcome.'''
class WaitForMsgStateX(smach.State):
    def __init__(self, topic, msg_type, additional_output_keys=[]):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted'],  output_keys=additional_output_keys)
        self.mutex = threading.Lock()
        self.msg = None
        self.subscriber = rospy.Subscriber(topic, msg_type, self._callback)

    def _callback(self, msg):
        self.mutex.acquire()
        self.msg = msg
        self.mutex.release()

    '''returns the message or None, not an outcome'''
    def waitForMsg(self):
        print 'Waiting for message...'
        # wait for a maximum of .. seconds
        for i in range(0, 30*100):
#            print 'gg_testing'
            self.mutex.acquire()
            if self.msg != None:
                print 'Got message.'
                message = self.msg
                self.msg = None
                return message
#                return 'succeeded'
            self.mutex.release()
            rospy.sleep(.1)
        
        print 'Timeout!'
        return None
        #return 'aborted'


class WaitForGoalStateX(WaitForMsgStateX):
    def __init__(self):
        WaitForMsgStateX.__init__(self, '/move_base_task/goal', PoseStamped, additional_output_keys=['x', 'y', 'yaw'])

    def execute(self, ud):
        msg = WaitForMsgStateX.waitForMsg(self)
        if msg == None:
            return 'aborted' 
        else:
            ud.x = msg.pose.position.x
            ud.y = msg.pose.position.y
            (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(pose_orientation_to_quaternion(msg.pose.orientation))
            ud.yaw = yaw
            return 'succeeded'




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






class HasMovedState(State):
    def _getXY(self):
        x,y,yaw = util.get_current_robot_position_in_odom_frame();
        return x,y
    
    def __init__(self, minimumDistance):
        smach.State.__init__(self, outcomes=['movement_exceeds_distance', 'movement_within_distance'])
        util.init_transform_listener()
        self.minimumDistance = minimumDistance
        self.lastX, self.lastY = self._getXY()

    def execute(self, userdata):
        currentX, currentY = self._getXY()
        currentDistance = math.sqrt(math.pow(currentX, 2) + math.pow(currentY, 2))
        rospy.logdebug("currentXY: %f,%f lastXY: %f,%f currentDistance: %f minimumDistance: %f", self.lastX, self.lastY, currentX, currentY, currentDistance, self.minimumDistance)
        if currentDistance >= self.minimumDistance:
            self.lastX = currentX
            self.lastY = currentY
            return 'movement_exceeds_distance'
        else:
            return 'movement_within_distance'


class ReadRobotPositionState(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['x', 'y', 'yaw'])
        util.init_transform_listener();

    def execute(self, userdata):
        userdata.x, userdata.y, userdata.yaw = util.get_current_robot_position_in_odom_frame();
        return 'succeeded'



if __name__ == '__main__':
    _test_WaitForGoalState()