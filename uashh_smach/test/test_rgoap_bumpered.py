'''
Created on Jul 2, 2013

@author: felix

needed ROS interfaces / mock ups:

# map
rosrun map_server map_server /home/felix/ros_workspace/recordings/2013-09_aula_alumni.yaml &
# move base server
mbnew &
# tfs
rosrun tf static_transform_publisher 0 0 0  0 0 0  map odom 40 &
rosrun tf static_transform_publisher 0 0 0  0 0 0  odom base_link  40 &

rostopic echo /bumper_reset &

# publisher for odometry, bumper status and joint states
'''
import rospy

from rgoap import Condition, MemoryCondition, Precondition, Goal
from rgoap.memory import MemoryChangeVarAction

from rgoap_ros import SMACHRunner

import uashh_smach.rgoap_subclasses as rgoap_subclasses

from uashh_smach.platform.move_base import position_tuple_to_pose



if __name__ == "__main__":

    rospy.init_node('rgoap_bumper_test', log_level=rospy.INFO)

    runner = SMACHRunner(rgoap_subclasses)

    Condition.add(MemoryCondition(runner.memory, 'memory.reminded_myself'))

    runner.memory.set_value('awareness', 0)
    runner.memory.set_value('arm_can_move', True)
    runner.memory.set_value('memory.reminded_myself', 333)

    rospy.loginfo("Waiting to let conditions represent reality...")
    rospy.loginfo("Remember to start topic publishers so conditions make sense instead of None!")
    rospy.sleep(2)
    Condition.initialize_worldstate(runner.worldstate)
    rospy.loginfo("worldstate now is: %s", runner.worldstate)

    runner.actions.add(MemoryChangeVarAction(runner.memory, 'memory.reminded_myself', 333, 555))


    goal = Goal([Precondition(Condition.get('robot.pose'), position_tuple_to_pose(1, 0, 0)),
                 Precondition(Condition.get('memory.reminded_myself'), 555)])

    start_node = runner.update_and_plan(goal, tries=2, introspection=True)

    rospy.loginfo("start_node: %s", start_node)


    if start_node is None:
        rospy.logwarn("No plan found! Check your ROS graph!")
    else:
        runner.execute_as_smach(start_node, introspection=True)


    rospy.sleep(20)

