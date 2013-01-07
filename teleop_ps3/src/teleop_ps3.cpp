#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <metralabs_ros/idAndFloat.h>

#include <math.h>


// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19



#define DEG_TO_RAD(d)	((d)*M_PI/180.0)
#define RAD_TO_DEG(r)	((r)*180.0/M_PI)


class teleop_ps3
{
public:
	teleop_ps3();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;

	ros::Subscriber joy_sub_;
	ros::Publisher vel_pub_, arm_pub_, ack_all_joints_pub_, arm_emergency_pub_, joints_position_pub_, gripper_pub_, smach_enable_pub_;

	const static int axis_speed = PS3_AXIS_STICK_LEFT_UPWARDS;
	const static int axis_turn = PS3_AXIS_STICK_LEFT_LEFTWARDS;
	const static int axis_turbo = PS3_AXIS_BUTTON_REAR_RIGHT_2;
	const static int axis_turbo_second = PS3_AXIS_BUTTON_REAR_LEFT_2;
	const static int button_deadman = PS3_BUTTON_REAR_RIGHT_1;
	const static int button_deadman_second = PS3_BUTTON_REAR_LEFT_1;
	const static int button_turbo = PS3_BUTTON_REAR_RIGHT_2;

	const static int button_modifier_config = PS3_BUTTON_START;
	const static int button_joints_ack_all = PS3_BUTTON_ACTION_SQUARE;
	const static int button_joints_go_pose = PS3_BUTTON_ACTION_TRIANGLE;
	const static int button_emergency = PS3_BUTTON_ACTION_CIRCLE;

	const static int button_modifier_smach = PS3_BUTTON_SELECT;
	const static int button_smach_enable = PS3_BUTTON_ACTION_CROSS;
	const static int button_smach_disable = PS3_BUTTON_ACTION_SQUARE;

	const static int axis_arm_pitch = PS3_AXIS_STICK_RIGHT_UPWARDS;
	const static int axis_arm_yaw = PS3_AXIS_STICK_RIGHT_LEFTWARDS;
	const static int axis_gripper_close = PS3_AXIS_BUTTON_CROSS_DOWN;
	const static int axis_gripper_open = PS3_AXIS_BUTTON_CROSS_UP;

	constexpr static double speed = 0.3; // .2 .5
	constexpr static double turn = 0.7; // .5 1
	constexpr static double joint_speed_pitch = 0.87;	// joint 4
	constexpr static double joint_speed_yaw = 0.43;		// joint 0
//	constexpr static double gripper_speed = 0.08;
	constexpr static double gripper_step = 0.01;
	constexpr static double gripper_step_delay = 0.05;
	constexpr static double velocity_epsilon = 0.001;
};


teleop_ps3::teleop_ps3()
{
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 1, &teleop_ps3::joyCallback, this);

	vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 3);
	arm_pub_ = nh_.advertise<geometry_msgs::Twist>("/moveArmVelocity", 3);
	ack_all_joints_pub_ = nh_.advertise<std_msgs::Bool>("/ackAll", 1);
	arm_emergency_pub_ = nh_.advertise<std_msgs::Bool>("/emergency", 1);
	joints_position_pub_ = nh_.advertise<sensor_msgs::JointState>("/schunk/target_pc/joint_states", 1);
	gripper_pub_ = nh_.advertise<metralabs_ros::idAndFloat>("/movePosition", 1);
	smach_enable_pub_ = nh_.advertise<std_msgs::Bool>("/enable_smach", 1, true);

}

void teleop_ps3::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	/// DEBUGGING

//	int i;
//	for(i=0; i < 16; i++)
//		ROS_INFO_STREAM("axis   " << i << " " << joy->axes[i]);
//	for(i=0; i < joy->buttons.size(); i++)
//		ROS_INFO_STREAM("button " << i << " " << joy->buttons[i]);


	/// base move control

	geometry_msgs::TwistPtr velocity_msg (new geometry_msgs::Twist);
	static bool sent_zero_last_time = false;
	if(joy->buttons[button_deadman] || joy->buttons[button_deadman_second] || joy->buttons[button_turbo]) {
		double speedfactor = 1 + (-joy->axes[axis_turbo]) + (-joy->axes[axis_turbo_second]); // up to thrice the speed if turbo
		velocity_msg->linear.x = joy->axes[axis_speed] * speed * speedfactor;
		velocity_msg->angular.z = joy->axes[axis_turn] * turn * speedfactor;

		ROS_DEBUG("effective linear x = %f, effective angular z = %f", velocity_msg->linear.x, velocity_msg->angular.z);
		// do not repeat zero'd messages on idle to not disturb other publishers
		if(fabs(velocity_msg->linear.x) <= velocity_epsilon && fabs(velocity_msg->angular.z) <= velocity_epsilon) {
			velocity_msg->linear.x = 0;
			velocity_msg->angular.z = 0;
			if(sent_zero_last_time == false) {
				sent_zero_last_time = true;
				vel_pub_.publish(velocity_msg);
				ROS_DEBUG("Sent zero'd.");
			}
			else
				ROS_DEBUG("Sent nothing.");
		}
		else {
			sent_zero_last_time = false;
			vel_pub_.publish(velocity_msg);
			ROS_DEBUG("Sent values.");
		}
	}
	else {
		// leave message zero'd
		// do not repeat zero'd messages on idle to not disturb other publishers
		if(sent_zero_last_time == false) {
			sent_zero_last_time = true;
			vel_pub_.publish(velocity_msg);
			ROS_DEBUG("Sent zero'd.");
		}
		else
			ROS_DEBUG("Sent nothing.");
	}


	/// arm move control

	if(joy->buttons[button_emergency]) {
		std_msgs::Bool booldummy;
		arm_emergency_pub_.publish(booldummy);
	}
	else if(joy->buttons[button_deadman] || joy->buttons[button_deadman_second]) {
		/// arm
		geometry_msgs::TwistPtr arm_msg (new geometry_msgs::Twist);
		arm_msg->angular.y = joy->axes[axis_arm_pitch] * joint_speed_pitch;
		arm_msg->angular.z = -joy->axes[axis_arm_yaw] * joint_speed_yaw;
		arm_pub_.publish(arm_msg);

		/// gripper
		// timed gate
		static ros::Time lastAction = ros::Time(0);
		if(ros::Time::now() - lastAction > ros::Duration(gripper_step_delay) ){
			lastAction = ros::Time::now();

			// static vars
			static float new_gripper_value = 0.068;
			static metralabs_ros::idAndFloatPtr gripper_msg (new metralabs_ros::idAndFloat);
			gripper_msg->id = 5;

			// input
			float close_axis = - joy->axes[axis_gripper_close]; // converted to 0 to 1
			float open_axis  = - joy->axes[axis_gripper_open ]; // converted to 0 to 1

			// logic

			if(!close_axis && open_axis)
				new_gripper_value += gripper_step * open_axis;
			else if(close_axis && !open_axis)
				new_gripper_value -= gripper_step * close_axis;

			new_gripper_value = fmax(new_gripper_value, 0);
			new_gripper_value = fmin(new_gripper_value, 0.068);

			if(gripper_msg->value != new_gripper_value) {
				ROS_INFO_STREAM("new_gripper_value: " << new_gripper_value);
				gripper_msg->value = new_gripper_value;
				gripper_pub_.publish(gripper_msg);
			}
		}

		/*// gripper
		metralabs_ros::idAndFloatPtr gripper_msg (new metralabs_ros::idAndFloat);
		gripper_msg->id = 5;

		float close_axis = - joy->axes[axis_gripper_close]; // converted to 0 to 1
		float open_axis  = - joy->axes[axis_gripper_open ]; // converted to 0 to 1

		if(!close_axis && open_axis)
			gripper_msg->value = open_axis * gripper_speed;
		else if(close_axis && !open_axis)
			gripper_msg->value = close_axis * -gripper_speed;
		else
			gripper_msg->value = 0;
		gripper_pub_.publish(gripper_msg);
		*/
	}

	/// second layer arm buttons

	if(joy->buttons[button_modifier_config]) {
		if(joy->buttons[button_joints_ack_all]) {
			std_msgs::Bool booldummy;
			ack_all_joints_pub_.publish(booldummy);
		}

		static bool button_blocked = false;
		if(joy->buttons[button_joints_go_pose]) {
			if(!button_blocked) {
				button_blocked = true;

				sensor_msgs::JointState::Ptr joint_msg (new sensor_msgs::JointState);
	//			std::string names[] = {"DH_0_1", "DH_1_2", "DH_2_3", "DH_3_4", "DH_4_5"};
	//			double values[] = {0, 0.5, 0.5, 1.5, 0};
	//			joint_msg->name.resize(5);
	//			joint_msg->position.resize(5);
	//			for(int i=0; i<5; i++) {
	//				joint_msg->name[i] = names[i];
	//				joint_msg->position[i] = values[i];
	//			}
				joint_msg->name = {"DH_1_2", "DH_2_3", "DH_3_4", "DH_4_5", "DH_5_6"};
				joint_msg->position = {0, 0.5, 0.5, -1.6, 0};
				joint_msg->velocity.assign(5, 0.3);
				joints_position_pub_.publish(joint_msg);
			}
		}
		else
			button_blocked = false;
	}


	/// second layer arm buttons

	if(joy->buttons[button_modifier_smach]) {
		std_msgs::Bool enable_msg;
		if(joy->buttons[button_smach_disable]) {
			enable_msg.data = false;
			smach_enable_pub_.publish(enable_msg);
		}
		else if(joy->buttons[button_smach_enable]) {
			enable_msg.data = true;
			smach_enable_pub_.publish(enable_msg);
		}

	}
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "teleop_ps3");
	ROS_INFO("Starting ps3 teleop converter, take care of your controller now...");
	teleop_ps3 teleop_ps3;
	ros::spin();
}
