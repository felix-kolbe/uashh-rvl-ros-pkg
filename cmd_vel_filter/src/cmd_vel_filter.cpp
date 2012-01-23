#include <stdio.h>
using namespace std;

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define LEN 3 // filter history length in msg count
#define TIMEOUT_SECS  2 // after what timeout ignore old filter history


ros::Publisher pub;

void callback(const geometry_msgs::TwistConstPtr& newInput)
{

	static double hist[LEN][2], sum[2], out[2];
	static int index = LEN;

	static ros::Time lastInput = ros::Time::now();
	static int fillLevel = 0;

	// point to next
	++index %= LEN;
	// increment fillLevel and limit it to LEN
	if(fillLevel < LEN)
		++fillLevel;

//	cout << "index: "<< index << " fill: "<<fillLevel << endl;
	// subtract last entry from sum
	for (int t=0; t<2; ++t)
		sum[t] -= hist[index][t];

	// insert new value
	hist[index][0] = newInput->linear.x;
	hist[index][1] = newInput->angular.z;
	// add new value
	for (int t=0; t<2; ++t)
		sum[t] += hist[index][t];

	// check if filter history is to old
	if(ros::Time::now() > lastInput + ros::Duration(TIMEOUT_SECS) || fillLevel < LEN)
		// forward new
		for (int t=0; t<2; ++t)
			out[t] = hist[index][t];
	else
		// calc avg
		for (int t=0; t<2; ++t) {
			out[t] = sum[t] / LEN;
		}

	// save timestamp
	lastInput = ros::Time::now();

	// send msg
	geometry_msgs::TwistPtr output = boost::make_shared<geometry_msgs::Twist>();
	output->linear.x = out[0];
	output->angular.z = out[1];
	pub.publish(output);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cmd_vel_filter");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("/cmd_vel_mb", 1, callback);
	pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, false);

	printf("I'm on air!");
	ros::spin();
	printf("I'm out! No error.");
	return 0;
}
