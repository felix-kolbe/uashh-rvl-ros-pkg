#include <math.h>

#include <ros/ros.h>

#include <metralabs_msgs/IDAndFloat.h>
#include <sensor_msgs/JointState.h>
#include <joint_motion_service/move_joints_service.h>


#define EPSILON_RADIAN 	0.0001
#define SLEEP_TIME_SECS	0.05
#define TIMEOUT_SECS	50


ros::Publisher pubMovePos;
sensor_msgs::JointStateConstPtr lastJointState;


void jointStatesCallback(const sensor_msgs::JointStateConstPtr& newState)
{
	lastJointState = newState;
}

bool serviceCallback( joint_motion_service::move_joints_service::Request &request, joint_motion_service::move_joints_service::Response &response)
{
	ROS_INFO("Service called!");

	if(lastJointState == 0) {
		ROS_ERROR("Joint move service receives no joint states!");
		response.positions_ok = false;
		return true;
	}

	// send wanted positions
	for (uint joint = 0; joint < request.joint_ids.size(); ++joint) {
		metralabs_msgs::IDAndFloatPtr output = boost::make_shared<metralabs_msgs::IDAndFloat>();
		output->id = request.joint_ids.at(joint);
		output->value = request.positions.at(joint);
		pubMovePos.publish(output);

		ros::Duration(SLEEP_TIME_SECS).sleep();
	}

	// wait for joints to reach position
	ros::Time timeoutTime = ros::Time::now() + ros::Duration(TIMEOUT_SECS);
	bool allPositionsReached;
	do {
		// check for timeout, arm to slow or not moving anymore
		if(ros::Time::now() > timeoutTime) {
			ROS_ERROR("Move joints service timed out! Stopping joint.");
			response.positions_ok = false;
			return true;
		}

		// sleep while arm moves a bit
		ros::Duration(SLEEP_TIME_SECS).sleep();

		// check if shutdown requested (e.g. Ctrl-C)
		if(ros::isShuttingDown())
			return false;

		// let the newest joint state message to be received
		ros::spinOnce();

		// check all current and wanted joint positions
		allPositionsReached = true;
		for (uint i = 0; i < request.joint_ids.size(); ++i) {
			uint id = request.joint_ids.at(i);
			float wishPosition = request.positions.at(i);
			float lastPosition = lastJointState->position.at(id);
			float deltaPosition = wishPosition - lastPosition;
			ROS_DEBUG("delta=%f", deltaPosition);
			if(fabs(deltaPosition) > EPSILON_RADIAN) {
				allPositionsReached = false;
				break;
			}
		}
	} while(!allPositionsReached);

	response.positions_ok = true;
	ROS_INFO("Service done!");
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_motion_service");
	ros::NodeHandle nh;

	pubMovePos = nh.advertise<metralabs_msgs::IDAndFloat>("/move_position", 1, false);
	ros::Subscriber subJointStates = nh.subscribe("/schunk/pre_mimic_joint_states", 1, jointStatesCallback);
	ros::ServiceServer service = nh.advertiseService("/move_joints_service", serviceCallback);

	ROS_INFO("Joint motion service running.");
	ros::spin();
	return 0;
}
