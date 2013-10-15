#!/usr/bin/env python

""" This file generates easy to use smach states needed to move the individual robot joints.
For this the joint_motion_service from our service is needed to be running.

For the gripper an even easier to use pre defined method is provided.
For other joints, move_arm.py should be used, as the joint_motion_service does not check for collisions.
 """

import roslib; roslib.load_manifest('uashh_smach')

import rospy

from smach_ros import ServiceState

from joint_motion_service.srv import move_joints_service, move_joints_serviceRequest


GRIPPER_ID = 5


class MoveJointsServiceState(ServiceState):
    """Subclass extending ServiceState to react on the service response data."""
    def execute(self, userdata):
        srv_out = ServiceState.execute(self, userdata)
        rospy.logdebug('srv_out was: ' + srv_out + ', response.positions_ok is: %r' % self._response.positions_ok)
#        rospy.loginfo('srv_out was: ' + srv_out + ', userdata[\'positions_ok\'] is: %r' % userdata['positions_ok'])
#        rospy.loginfo('srv_out was: ' + srv_out + ', userdata.positions_ok is: %r' % userdata.positions_ok)

        # if srv call was successfull but the joint motion wasn't, override return value
        if srv_out == 'succeeded' and self._response.positions_ok != True:
            return 'aborted'

        return srv_out


def get_move_gripper_state(grab_width):
    return MoveJointsServiceState('move_joints_service', move_joints_service,
                                  request=move_joints_serviceRequest([5], [grab_width])
                                  )


#def getMoveArmToZerosState(): do not use - no collision checking
#    return MoveJointsServiceState('move_joints_service', move_joints_service,
#                                  request=move_joints_serviceRequest([0,1,2,3,4], [0,0,0,0,0])
#                                  )
