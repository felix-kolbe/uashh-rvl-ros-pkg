'''
RGOAPTestPreconditionEffectSymmetry

Created on Jul 2, 2013
@author: felix

needed ROS mock ups:

rosrun map_server map_server /home/felix/ros_workspace/recordings/2013-09_aula_alumni.yaml &
mbnew & # move base server
rosrun tf static_transform_publisher 0 0 0  0 0 0  map base_link 40 &
rosrun tf static_transform_publisher 0 0 0  0 0 0  base_link  odom  40 &
rostopic echo /bumper_reset &
'''

import rospy

from rgoap.common import Condition, Precondition, Goal
from rgoap.runner import Runner

import rgoap.config_scitos as config_scitos


if __name__ == "__main__":

    rospy.init_node('rgoap_bumper_test', log_level=rospy.INFO)

    runner = Runner(config_scitos)

    print 'Waiting to let conditions represent reality...'
    print 'Remember to start topic publishers so conditions make sense instead of None!'
    rospy.sleep(2)
    Condition.initialize_worldstate(runner.worldstate)
    print 'worldstate now is: ', runner.worldstate


    goal = Goal([Precondition(Condition.get('robot.bumpered'), False)])

    start_node = runner.update_and_plan(goal, introspection=True)

    print 'start_node: ', start_node


    if start_node is None:
        print 'No plan found! Check your ROS graph!'
    else:
        runner.execute_as_smach(start_node, introspection=True)


    rospy.sleep(20)

