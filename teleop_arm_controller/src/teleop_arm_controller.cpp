#include <cmath>

#include <ros/ros.h>

#include <metralabs_msgs/IDAndFloat.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>


#define SLEEP_TIME_SECS 0.10
#define TIMEOUT_SECS    0.25

#define YAW_JOINT_ID    0
#define PITCH_JOINT_ID  3

#define MAX_JOINT_SPEED	0.4


#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define DEG_TO_RAD(d)	((d)*M_PI/180.0)
#define RAD_TO_DEG(r)	((r)*180.0/M_PI)


ros::Publisher pubMoveVel;
ros::Time lastMessage;
sensor_msgs::JointStateConstPtr lastJointState;
bool initiatedMovement;


bool checkVelocityValid(int joint, double velocity, double currentAngle)
{
  if (joint == YAW_JOINT_ID)
  {
    if ((velocity > 1e-6 && currentAngle >= DEG_TO_RAD(170)) || (velocity < -1e-6 && currentAngle <= DEG_TO_RAD(-170)))
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else if (joint == PITCH_JOINT_ID)
  {
    if ((velocity > 1e-6 && currentAngle >= DEG_TO_RAD(-10)) || (velocity < -1e-6 && currentAngle <= DEG_TO_RAD(-120)))
//    if ((velocity > 1e-6 && currentAngle >= DEG_TO_RAD(50)) || (velocity < -1e-6 && currentAngle <= DEG_TO_RAD(-120)))
    {
      return false;
    }
    else
    {
      return true;
    }
  }
  else
  {
    return false;
  }
}

void sendVelocity(int joint, double velocity)
{
  metralabs_msgs::IDAndFloatPtr output = boost::make_shared<metralabs_msgs::IDAndFloat>();
  output->id = joint;
  output->value = velocity;
  pubMoveVel.publish(output);
}

void jointStatesCallback(const sensor_msgs::JointStateConstPtr& newState)
{
  lastJointState = newState;
  if (initiatedMovement)
  {
    //ROS_INFO("Got joint states and i am moving the arm");
    if (!checkVelocityValid(YAW_JOINT_ID, lastJointState->velocity.at(YAW_JOINT_ID), lastJointState->position.at(YAW_JOINT_ID)))
    {
      sendVelocity(YAW_JOINT_ID, 0);
    }
    if (!checkVelocityValid(PITCH_JOINT_ID, lastJointState->velocity.at(PITCH_JOINT_ID), lastJointState->position.at(PITCH_JOINT_ID)))
    {
      sendVelocity(PITCH_JOINT_ID, 0);
    }
  }
//  else
//  {
//    ROS_INFO("Got joint states and i am NOT moving the arm");
//  }
}

void moveVelocityCallback(const geometry_msgs::TwistConstPtr& moveVelocity)
{
  if(!lastJointState)
  {
	static bool printed_warn_before = false;
	if(!printed_warn_before)
	{
	  printed_warn_before = true;
	  ROS_WARN("Ignoring move velocity command until joint state message received!");
	}
  }
  else
  {
    lastMessage = ros::Time::now();
    double yaw = MAX(MIN(moveVelocity->angular.z, MAX_JOINT_SPEED), -MAX_JOINT_SPEED);
    double pitch = MAX(MIN(moveVelocity->angular.y, MAX_JOINT_SPEED), -MAX_JOINT_SPEED);

    bool moving = false;

    //ROS_INFO("Yaw: %f  Pos: %f  CurVel: %f", yaw, lastJointState->position.at(YAW_JOINT_ID), lastJointState->velocity.at(YAW_JOINT_ID));
    if (std::abs(yaw) > 1e-4 && checkVelocityValid(YAW_JOINT_ID, yaw, lastJointState->position.at(YAW_JOINT_ID)))
    {
      if (std::abs(yaw - lastJointState->velocity.at(YAW_JOINT_ID)) > 2e-2)
      {
        sendVelocity(YAW_JOINT_ID, yaw);
        //ROS_INFO("Started yaw movement: %f", yaw);
      }
      moving = true;
    }
    else
    {
      if (std::abs(lastJointState->velocity.at(YAW_JOINT_ID)) > 1e-4)
      {
        sendVelocity(YAW_JOINT_ID, 0);
      }
    }

    //ROS_INFO("Pitch: %f  Pos: %f  CurVel: %f", pitch, lastJointState->position.at(PITCH_JOINT_ID), lastJointState->velocity.at(PITCH_JOINT_ID));
    if (std::abs(pitch) > 1e-6 && checkVelocityValid(PITCH_JOINT_ID, pitch, lastJointState->position.at(PITCH_JOINT_ID)))
    {
      if (std::abs(pitch - lastJointState->velocity.at(PITCH_JOINT_ID)) > 2e-2)
      {
        sendVelocity(PITCH_JOINT_ID, pitch);
        //ROS_INFO("Started pitch movement: %f", pitch);
      }
      moving = true;
    }
    else
    {
      if (std::abs(lastJointState->velocity.at(PITCH_JOINT_ID)) > 1e-4)
      {
        sendVelocity(PITCH_JOINT_ID, 0);
      }
    }
    initiatedMovement = moving;
  }
}


//bool serviceCallback( joint_motion_service::move_joints_service::Request &request, joint_motion_service::move_joints_service::Response &response)
//{
//        ROS_INFO("Service called!");
//
//        if(lastJointState == 0) {
//                ROS_ERROR("Joint move service receives no joint states!");
//                response.positions_ok = false;
//                return true;
//        }
//
//        // send wanted positions
//        for (uint joint = 0; joint < request.joint_ids.size(); ++joint) {
//                metralabs_msgs::IDAndFloatPtr output = boost::make_shared<metralabs_msgs::IDAndFloat>();
//                output->id = request.joint_ids.at(joint);
//                output->value = request.positions.at(joint);
//                pubMoveVel.publish(output);
//
//                ros::Duration(SLEEP_TIME_SECS).sleep();
//        }
//
//        // wait for joints to reach position
//        ros::Time timeoutTime = ros::Time::now() + ros::Duration(TIMEOUT_SECS);
//        bool allPositionsReached;
//        do {
//                // check for timeout, arm to slow or not moving anymore
//                if(ros::Time::now() > timeoutTime) {
//                        ROS_ERROR("Move joints service timed out! Stopping joint.");
//                        response.positions_ok = false;
//                        return true;
//                }
//
//                // sleep while arm moves a bit
//                ros::Duration(SLEEP_TIME_SECS).sleep();
//
//                // check if shutdown requested (e.g. Ctrl-C)
//                if(ros::isShuttingDown())
//                        return false;
//
//                // let the newest joint state message to be received
//                ros::spinOnce();
//
//                // check all current and wanted joint positions
//                allPositionsReached = true;
//                for (uint i = 0; i < request.joint_ids.size(); ++i) {
//                        uint id = request.joint_ids.at(i);
//                        float wishPosition = request.positions.at(i);
//                        float lastPosition = lastJointState->position.at(id);
//                        float deltaPosition = wishPosition - lastPosition;
//                        ROS_DEBUG("delta=%f", deltaPosition);
//                        if(fabs(deltaPosition) > EPSILON_RADIAN) {
//                                allPositionsReached = false;
//                                break;
//                        }
//                }
//        } while(!allPositionsReached);
//
//        response.positions_ok = true;
//        ROS_INFO("Service done!");
//        return true;
//}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_arm_controller");
  ros::NodeHandle nh;

  pubMoveVel = nh.advertise<metralabs_msgs::IDAndFloat>("/move_velocity", 1, false);
  ros::Subscriber subJointStates = nh.subscribe("/schunk/pre_mimic_joint_states", 1, jointStatesCallback);
  ros::Subscriber subArmTwist = nh.subscribe("/moveArmVelocity", 1, moveVelocityCallback);

  ROS_INFO("Teleop arm controller online.");
  while (ros::ok())
  {
    ros::spinOnce();
    if (initiatedMovement)
    {
      if (ros::Time::now() > lastMessage + ros::Duration(TIMEOUT_SECS))
      {
        // no message received for the timeout period. stopping motion
        sendVelocity(YAW_JOINT_ID, 0);
        sendVelocity(PITCH_JOINT_ID, 0);
        initiatedMovement = false;
        ROS_INFO("Message timeout. Motion halted!");
      }
    }
    ros::Duration(SLEEP_TIME_SECS).sleep();
  }
  return 0;
}
