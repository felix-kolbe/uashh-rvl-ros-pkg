#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <string>
#include <vector>

int main (int argc, char **argv)
{
	ros::init(argc, argv, "follow_joint_trajectory_action_tester");

	// create the action client
	// true causes the client to spin its own thread
	actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("schunk/follow_joint_trajectory", true);

	ROS_INFO("Waiting for action server..");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server available, sending goal..");
	// send a goal to the action
	control_msgs::FollowJointTrajectoryGoal goal;

	trajectory_msgs::JointTrajectory& traj = goal.trajectory;

	std::vector<std::string> names = {"DH_5_6"};
	traj.joint_names = names;

	trajectory_msgs::JointTrajectoryPoint p1, p2, p3, p4;

	if(true) { // acceleration test
		p1.accelerations = {0.1};
		p1.velocities = {0.8};
		p1.positions = {3};
		p1.time_from_start = ros::Duration(5);

		p2.accelerations = {3};
		p2.velocities = {0.8};
		p2.positions = {-3};
		p2.time_from_start = ros::Duration(5*2);

		p3.accelerations = {0.1};
		p3.velocities = {0.8};
		p3.positions = {3};
		p3.time_from_start = ros::Duration(5*3);

		p4.accelerations = {3};
		p4.velocities = {0.8};
		p4.positions = {-3};
		p4.time_from_start = ros::Duration(5*4);

		traj.points = {p1, p2, p3, p4};

	} else { // velocity test
		p1.accelerations = {0.1};
		p1.velocities = {0.2};
		p1.positions = {1};
		p1.time_from_start = ros::Duration(5);

		p2.accelerations = {0.1};
		p2.velocities = {0.2};
		p2.positions = {0};
		p2.time_from_start = ros::Duration(5*2);

		p3.accelerations = {0.1};
		p3.velocities = {0.4};
		p3.positions = {-1};
		p3.time_from_start = ros::Duration(5*3);

		p4.accelerations = {0.1};
		p4.velocities = {0.4};
		p4.positions = {0};
		p4.time_from_start = ros::Duration(5*4);

		traj.points = {p1, p2, p3, p4};
	}


	ac.sendGoal(goal);

	ROS_INFO("Sleeping..");
	ros::Duration(3).sleep();

	ROS_INFO("Cancelling goal..");
	ac.cancelAllGoals();

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(10.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	//exit
	return 0;
}
