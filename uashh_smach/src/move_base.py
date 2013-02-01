#!/usr/bin/env python

""" This file generates easy to use smach states needed to move the robot base. """

import roslib; roslib.load_manifest('uashh_smach')

import rospy
import tf

import math
import random

import smach
import smach_ros
from smach import State, Sequence
from smach_ros import ServiceState, SimpleActionState

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion 

import util
from util import WaitForMsgState



def pose_orientation_to_quaternion(msg):
    return [msg.x, msg.y, msg.z, msg.w]



    
def get_move_base_in_map_state(x, y):
    return get_move_base_state("/map", x, y)

def get_move_base_in_odom_state(x, y):
    return get_move_base_state("/odom", x, y)

def get_move_base_random_state():
    '''Note: each state returned is only randomized once at initialization and then static.'''
    radius = random.random()*2 + 1  # 1-3 m
    yaw = random.random()*util.TAU*3/4 - util.TAU*3/8    # +-135 deg
    
    return get_move_base_state("/base_link", math.cos(yaw)*radius, math.sin(yaw)*radius, yaw)
    
def get_move_base_state(frame='/map', x=0, y=0, yaw=0):
    '''Return a MoveBaseGoal state which goal parameters are given via parameters at setup time.'''
    print "new goal: ", x, y, yaw
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



class MoveBaseState(SimpleActionState):
    """Calls a move_base action server with the goal (x, y, yaw) from userdata.
    
    frame: defaults to /map
    """
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

        
class CalcRandomGoalState(State):
    """Return a random (x,y,yaw) tuple via userdata.
    
    (x,y) lies in the direction range of +135 degrees.
    Radius range defaults to 1-3 m.
    """
    def __init__(self, radius_min=1, radius_max=3):
        State.__init__(self, outcomes=['succeeded'], output_keys=['x', 'y', 'yaw'])
        self.radius_min = radius_min
        self.radius_max = radius_max
    
    def execute(self, ud):
        radius = random.random()*(self.radius_max - self.radius_min) + self.radius_min
        yaw = random.random() * util.TAU*3/4 - util.TAU*3/8    # +-135 deg
        
        ud.x = math.cos(yaw) * radius
        ud.y = math.sin(yaw) * radius
        ud.yaw = yaw
        return 'succeeded'


def get_random_goal_smach(frame):
    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'], connector_outcome = 'succeeded') 
    
    sq.userdata.x = 0
    sq.userdata.y = 0
    sq.userdata.yaw = 0
    
    with sq: 
        # implicit usage of above userdata
        Sequence.add("CALC_RANDOM_GOAL", CalcRandomGoalState())
        Sequence.add("MOVE_RANDOM_GOAL", MoveBaseState(frame))
    
    return sq




class WaitForGoalState(WaitForMsgState):
    def __init__(self):
        WaitForMsgState.__init__(self, '/move_base_task/goal', PoseStamped, self._msg_cb, output_keys=['x', 'y', 'yaw'])

    def _msg_cb(self, msg, ud):
        ud.x = msg.pose.position.x
        ud.y = msg.pose.position.y
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(pose_orientation_to_quaternion(msg.pose.orientation))
        ud.yaw = yaw




class HasMovedState(State):
    """Return whether the robot moved beyond a given distance in a given frame since the last exceeding check.""" 
    def _getXY(self):
        x,y,yaw = util.get_current_robot_position(self.frame);
        return x,y
    
    def __init__(self, minimumDistance, frame='/map'):
        smach.State.__init__(self, outcomes=['movement_exceeds_distance', 'movement_within_distance'])
        util.init_transform_listener()
        self.minimumDistance = minimumDistance
        self.frame = frame
        self.lastX, self.lastY = self._getXY()

    def execute(self, userdata):
        currentX, currentY = self._getXY()
        currentDistance = math.sqrt(math.pow(currentX, 2) + math.pow(currentY, 2))
        rospy.logdebug("currentXY: %f,%f lastXY: %f,%f currentDistance: %f minimumDistance: %f", 
                       self.lastX, self.lastY, currentX, currentY, currentDistance, self.minimumDistance)
        if currentDistance >= self.minimumDistance:
            self.lastX = currentX
            self.lastY = currentY
            return 'movement_exceeds_distance'
        else:
            return 'movement_within_distance'


class ReadRobotPositionState(State):
    """Return the current robot position in the given frame via userdata.
    
    frame: defaults to /map
    """
    def __init__(self, frame='/map'):
        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['x', 'y', 'yaw'])
        self.frame = frame
        util.init_transform_listener();

    def execute(self, userdata):
        userdata.x, userdata.y, userdata.yaw = util.get_current_robot_position(self.frame);
        return 'succeeded'
