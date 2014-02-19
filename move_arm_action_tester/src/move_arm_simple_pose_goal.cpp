#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <arm_navigation_msgs/MoveArmAction.h>
#include <arm_navigation_msgs/SimplePoseConstraint.h>
#include <arm_navigation_msgs/utils.h>

#include <tf/transform_datatypes.h>

float parseArg(char* arg) {

	if(!strcmp(arg, "M_PI"))
		return M_PI;
	if(!strcmp(arg, "M_PI_2"))
		return M_PI_2;
	if(!strcmp(arg, "M_PI_4"))
		return M_PI_4;
	if(!strcmp(arg, "-M_PI"))
		return -M_PI;
	if(!strcmp(arg, "-M_PI_2"))
		return -M_PI_2;
	if(!strcmp(arg, "-M_PI_4"))
		return -M_PI_4;
	else
		return atof(arg);

}


int main(int argc, char **argv){
	ros::init (argc, argv, "move_arm_pose_goal_test");
	ros::NodeHandle nh;
	actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction> move_arm("move_SchunkArm",true);
	move_arm.waitForServer();
	ROS_INFO("Connected to server");
	arm_navigation_msgs::MoveArmGoal goalA;

	double	x = 0.5,
			y = 0.1,
			z = 0.5,
			R = -M_PI_2, // greifer 'gerade' drehen
			P = M_PI_2, // M_PI_2 negativ: greifer zeigt nach oben. und umgekehrt
			Y = 0;

	// 0 0 0.6   -M_PI_2  M_PI_4 0
	// 0.3 0 0.4   -M_PI_2  M_PI_2 0
	// 0.5 0 0.3   -M_PI_2  M_PI_2 0
	// 0.4 0.1 0.4   -M_PI_2  M_PI_2 -M_PI_2

	if(argc >= 1+6) { // cmd name + args
		x = parseArg(argv[1]);
		y = parseArg(argv[2]);
		z = parseArg(argv[3]);
		R = parseArg(argv[4]);
		P = parseArg(argv[5]);
		Y = parseArg(argv[6]);
	}

	ROS_INFO_STREAM("x="<<x<<" y="<<y<<" z="<<z<<" R="<<R<<" P="<<P<<" Y="<<Y);

	goalA.motion_plan_request.group_name = "SchunkArm";
	goalA.motion_plan_request.num_planning_attempts = 2;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

	arm_navigation_msgs::SimplePoseConstraint desired_pose;
	desired_pose.header.frame_id = "arm_base_link";
	desired_pose.link_name = "gripper";
	desired_pose.pose.position.x = x;
	desired_pose.pose.position.y = y;
	desired_pose.pose.position.z = z;

	double yaw = tan(desired_pose.pose.position.y / desired_pose.pose.position.x);
	ROS_INFO_STREAM("yaw="<<yaw);

	if(argc >= 1+7 && !strcmp(argv[7], "calcyaw")) {
		ROS_INFO("Using calculated yaw");
		Y = yaw;
	}

	desired_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(R, P, Y);

	ROS_INFO_STREAM(desired_pose.pose.orientation);


	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;

	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	if (nh.ok())
	{
		bool finished_within_time = false;
		move_arm.sendGoal(goalA);
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
