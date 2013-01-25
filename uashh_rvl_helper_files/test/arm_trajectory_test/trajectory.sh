#!/bin/bash
#[ INFO] [1318861698.591898154]: 0 is mapping to DH_1_2
#[ INFO] [1318861698.592026684]: 1 is mapping to DH_2_3
#[ INFO] [1318861698.592471477]: 2 is mapping to DH_3_4
#[ INFO] [1318861698.592571647]: 3 is mapping to DH_4_5
#[ INFO] [1318861698.593048541]: 4 is mapping to DH_5_6


rostopic echo -n 1 /schunk/trajectory/state


rostopic pub --once /schunk/trajectory/command trajectory_msgs/JointTrajectory -f trajectory_vel.yaml

#rostopic pub --once /schunk/trajectory/command trajectory_msgs/JointTrajectory -f trajectory_pos.yaml

rostopic echo -n 5 /schunk/trajectory/state


#$ rosmsg show trajectory_msgs/JointTrajectory
#Header header
  #uint32 seq
  #time stamp
  #string frame_id
#string[] joint_names
#trajectory_msgs/JointTrajectoryPoint[] points
  #float64[] positions
  #float64[] velocities
  #float64[] accelerations
  #duration time_from_start
