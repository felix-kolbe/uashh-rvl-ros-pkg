#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

class MovementFilter
{
private:
  ros::Publisher pub;
  tf::TransformListener tl;
  ros::NodeHandle nh;

  ros::Subscriber subPoints;

  std::string baseFrame;
  std::string sensorFrame;

  double noMoveSpan;

public:
  MovementFilter()
  {
    ros::NodeHandle pnh("~");

    // get params
    std::string inTopicName;
    std::string outTopicName;

    pnh.param<std::string>("in_topic", inTopicName, "/Camera/DepthRegistered/Points");
    pnh.param<std::string>("out_topic", outTopicName, inTopicName + "_noMotion");

    pnh.param<std::string>("base_frame", baseFrame, "/odom");
    pnh.param<std::string>("sensor_frame", sensorFrame, "/camera_link");

    pnh.param<double>("no_movement_span_seconds", noMoveSpan, 0.5);

    pub = nh.advertise<sensor_msgs::PointCloud2> (outTopicName, 1);

    subPoints = nh.subscribe(inTopicName, 1, &MovementFilter::pc2Callback, this);
    ROS_INFO("Finished constructor...");
  }

  void pc2Callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud)
  {
    tf::StampedTransform pastTf, msgTf, currentTf;

    try
    {
      tl.lookupTransform(sensorFrame, baseFrame, ros::Time::now() - ros::Duration(noMoveSpan), pastTf);
      ros::Time currentTime = ros::Time::now();
      tl.waitForTransform(sensorFrame, baseFrame, currentTime, ros::Duration(0.3), ros::Duration(0.01));
      tl.lookupTransform(sensorFrame, baseFrame, (*point_cloud).header.stamp, msgTf);
      tl.lookupTransform(sensorFrame, baseFrame, currentTime, currentTf);

      if (compareTf(pastTf, msgTf) && compareTf(msgTf, currentTf))
      {
        ROS_INFO("No motion detected, publishing message.");
        pub.publish(point_cloud);
      }
      else
      {
        ROS_INFO("Motion detected, dropping message.");
      }
    }
    catch (tf::TransformException &e)
    {
      ROS_INFO("Transform failed: %s", e.what());
    }
  }

  bool compareTf(tf::StampedTransform &tf1, tf::StampedTransform &tf2)
  {
    return (tf1.getBasis() == tf2.getBasis());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_movement_filter");
  MovementFilter mf;

  ros::spin();
  return 0;
}
