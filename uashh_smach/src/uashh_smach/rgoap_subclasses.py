'''
Created on Nov 26, 2013

@author: felix
'''
import roslib; roslib.load_manifest('uashh_smach')
import rospy

from rospy.service import ServiceException

import tf

from std_msgs.msg import Empty
from nav_msgs.msg import Path, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from metralabs_msgs.msg import ScitosG5Bumper

from rgoap import Condition, Precondition, VariableEffect, MemoryCondition
from rgoap import Action, Effect, Goal
from rgoap_ros import ROSTopicCondition, ROSTopicAction
from rgoap_smach import SMACHStateWrapperAction

from uashh_smach.manipulator.look_around import get_lookaround_smach
from uashh_smach.manipulator.move_arm import get_move_arm_to_joints_positions_state
from uashh_smach.platform.move_base import MoveBaseState, pose_orientation_to_quaternion


from uashh_smach.config_scitos import *



### list getter

def get_all_conditions(memory):
    return [
        # memory
        MemoryCondition(memory, 'arm_can_move', True),
        MemoryCondition(memory, 'awareness', 0),
        # ROS
        ROSTopicCondition('robot.pose', '/odom', Odometry, '/pose/pose'),
        ROSTopicCondition('robot.bumpered', '/bumper', ScitosG5Bumper, '/motor_stop'),
        ROSTopicCondition('robot.arm_folded', '/joint_states', JointState,
                          msgeval=lambda msg: check_joint_msg_matches_pose(msg, ARM_POSE_FOLDED_NAMED)),
        ROSTopicCondition('robot.arm_pose_floor', '/joint_states', JointState,
                          msgeval=lambda msg: check_joint_msg_matches_pose(msg, ARM_POSE_FLOOR_NAMED))
        ]


def get_all_actions(memory):
    return [
        # memory
        # ROS - pure actions
        ResetBumperAction(),
        # ROS - wrapped SMACH states
        MoveBaseAction(),
        LookAroundAction(),
        FoldArmAction(),
        MoveArmFloorAction()
        ]



def get_all_goals(memory):
    return [
        # memory
        # ROS
#        MoveAroundGoal(),
        LocalAwareGoal()
        ]



### Actions

class LookAroundAction(SMACHStateWrapperAction):
    def __init__(self):
        SMACHStateWrapperAction.__init__(
                    self, get_lookaround_smach(glimpse=True),
                    [Precondition(Condition.get('arm_can_move'), True)],
                    [VariableEffect(Condition.get('awareness'))])

    def _generate_variable_preconditions(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect is self._effects[0]
        # increase awareness by one
        precond_value = worldstate.get_condition_value(effect._condition) - 1
        return [Precondition(effect._condition, precond_value, None)]


class FoldArmAction(SMACHStateWrapperAction):
    def __init__(self):
        SMACHStateWrapperAction.__init__(
                self, get_move_arm_to_joints_positions_state(ARM_POSE_FOLDED),
                [Precondition(Condition.get('arm_can_move'), True),
                 # TODO: maybe remove necessary anti-effect-preconditions
                 # the currently available alternative would be to use a
                 # variable effect that can reach any value
                 Precondition(Condition.get('robot.arm_folded'), False)],
                [Effect(Condition.get('robot.arm_folded'), True)])


class MoveArmFloorAction(SMACHStateWrapperAction):
    def __init__(self):
        SMACHStateWrapperAction.__init__(
                self, get_move_arm_to_joints_positions_state(ARM_POSE_FLOOR),
                [Precondition(Condition.get('arm_can_move'), True),
                 # TODO: maybe remove necessary anti-effect-preconditions
                 # the currently available alternative would be to use a
                 # variable effect that can reach any value
                 Precondition(Condition.get('robot.arm_pose_floor'), False)],
                [Effect(Condition.get('robot.arm_pose_floor'), True)])


class ResetBumperAction(ROSTopicAction):
    def __init__(self):
        ROSTopicAction.__init__(
                self, '/bumper_reset', Empty,
                [Precondition(Condition.get('robot.bumpered'), True)],
                [Effect(Condition.get('robot.bumpered'), False)],
                msg_args=[])



class MoveBaseAction(SMACHStateWrapperAction):

    class CheckForPathVarEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(self, condition)
            self.service_topic = '/move_base/make_plan'
            self._service_proxy = rospy.ServiceProxy(self.service_topic, GetPlan)
            self._planned_paths_pub = rospy.Publisher('/task_planning/goal_paths', Path)

        def _is_reachable(self, value, start_value):
            request = GetPlanRequest()
            request.start.header.stamp = rospy.Time.now()
            request.start.header.frame_id = '/map'
            request.start.pose = start_value
            request.goal.header.stamp = rospy.Time.now()
            request.goal.header.frame_id = '/map'
            request.goal.pose = value
            request.tolerance = 0.3 # meters in x/y
            rospy.logdebug("%s sending request: %s", self, request)
            try:
                response = self._service_proxy(request)
                rospy.logdebug("%s received response: %s", self, response)
                response.plan.header.frame_id = '/map'
                self._planned_paths_pub.publish(response.plan)
                return len(response.plan.poses) > 0
            except ServiceException as e:
                rospy.logerr(e)
                return None

    def __init__(self):
        self._condition = Condition.get('robot.pose')
        self._check_path_vareffect = MoveBaseAction.CheckForPathVarEffect(self._condition)
        SMACHStateWrapperAction.__init__(self, MoveBaseState(),
                        [Precondition(Condition.get('robot.bumpered'), False),
                         Precondition(Condition.get('robot.arm_pose_floor'), True)
                        ],
                        [self._check_path_vareffect])

    def check_freeform_context(self):
        # TODO: cache freeform context?
        if not self.state._action_client.wait_for_server(rospy.Duration(1)):
            rospy.logwarn("%s context check: cannot access move_base action server"
                          % self.__class__.__name__)
            return False
        try:
            self._check_path_vareffect._service_proxy.wait_for_service(1)
        except rospy.exceptions.ROSException:
            rospy.logwarn("%s context check: cannot access %s service server"
                          % (self.__class__.__name__,
                             self._check_path_vareffect.service_topic))
            return False
        return True

    def _generate_variable_preconditions(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect._condition is self._condition
        precond_value = start_worldstate.get_condition_value(Condition.get('robot.pose'))
        return [Precondition(effect._condition, precond_value, None)]

    def translate_worldstate_to_userdata(self, next_worldstate, userdata):
        goal_pose = next_worldstate.get_condition_value(Condition.get('robot.pose'))
        (_roll, _pitch, yaw) = tf.transformations.euler_from_quaternion(
                        pose_orientation_to_quaternion(goal_pose.orientation))
        userdata.x = goal_pose.position.x
        userdata.y = goal_pose.position.y
        userdata.yaw = yaw



### Goals

class MoveToPoseGoal(Goal):
    tl = None

    def __init__(self, pose, frame, usability):
        if frame != '/map' and frame != 'map':
            if MoveToPoseGoal.tl is None:
                MoveToPoseGoal.tl = tf.TransformListener()
                rospy.sleep(1)
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = frame
            pose_stamped.pose = pose
            #tf.Transformer().waitForTransform('/map', frame)
            pose_stmpd_in_map = MoveToPoseGoal.tl.transformPose('/map', pose_stamped)
            pose = pose_stmpd_in_map.pose
        Goal.__init__(self, [Precondition(Condition.get('robot.pose'), pose)], usability)


class LocalAwareGoal(Goal):
    def __init__(self):
        Goal.__init__(self, [Precondition(Condition.get('awareness'), 1)], 0.6)


