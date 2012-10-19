// originally copied from pr2_arm_navigation_tutorials/src/move_arm_joint_goal.cpp

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <arm_navigation_msgs/MoveArmAction.h>

int main(int argc, char **argv){
	ros::init (argc, argv, "move_arm_joint_goal_test");
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_SchunkArm",true); //move_right_arm

	move_arm.waitForServer();
	ROS_INFO("Connected to server");

	arm_navigation_msgs::MoveArmGoal goalB;
	std::vector<std::string> names(5); // don't forget this number
	names[0] = "DH_1_2";
	names[1] = "DH_2_3";
	names[2] = "DH_3_4";
	names[3] = "DH_4_5";
	names[4] = "DH_5_6";

	goalB.motion_plan_request.group_name = "SchunkArm";
	goalB.motion_plan_request.num_planning_attempts = 1;
	goalB.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

	goalB.motion_plan_request.planner_id = std::string("");
	goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalB.motion_plan_request.goal_constraints.joint_constraints.resize(names.size());

	/* joint_constraints : Each joint constraints is specified with a joint_name,
	the goal position that we want the joint to reach and
	a tolerance above and below this position that we are willing to accept.
	Thus, the accepted range of tolerances is [position-tolerance_below,position+tolerance_above]
	from wiki */
	for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
	{
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = names[i];
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = 0.0;
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.01;
		goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.01;
	}
//	goalB.motion_plan_request.goal_constraints.joint_constraints[0].position = -2.0;

	if (nh.ok())
	{
		bool finished_within_time = false;
		move_arm.sendGoal(goalB);
		finished_within_time = move_arm.waitForResult(ros::Duration(200.0));
		if (!finished_within_time)
		{
			move_arm.cancelGoal();
			ROS_INFO("Timed out achieving goal A");
		}
		else
		{
			actionlib::SimpleClientGoalState state = move_arm.getState();
			bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			if(success)
				ROS_INFO("Action finished: %s",state.toString().c_str());
			else
				ROS_INFO("Action failed: %s",state.toString().c_str());
		}
	}
	ros::shutdown();
}
