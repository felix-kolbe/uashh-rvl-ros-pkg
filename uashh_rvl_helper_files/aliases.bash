HELPER_PKG=uashh_rvl_helper_files

alias aliases_edit='rosed '$HELPER_PKG' aliases.bash'
alias aliases_reload='source `rospack find $HELPER_PKG`/aliases.bash'


# config adjustments
alias ros_what='echo ROS_MASTER_URI=$ROS_MASTER_URI; echo ROS_ROOT=$ROS_ROOT; echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH'
alias ros_scitos='export ROS_MASTER_URI=http://scitos_w:11311 ; export PS1="'$PS1'"'
alias ros_local='export ROS_MASTER_URI=http://localhost:11311 ; export PS1="'$(echo "$PS1" | sed 's/\\\$ $//')'!\[\033[01;33m\]roslocal\[\033[00m\]$ "'


# all
alias uashh_all='roslaunch '$HELPER_PKG' uashh_all.launch'
alias scitos_all='mla & telearm & ps3 & rs & lsr & kinect & cm & diag_agg & jms & kmf & gmapp & mbnew & octomap & collmap & armnav &'

# hardware nodes
alias ml='roslaunch metralabs_ros scitos_haw_only_start.launch'
alias mla='roslaunch metralabs_ros scitos_haw_schunk_start.launch'
alias mla_mock='jsp'
alias rs='roslaunch '$HELPER_PKG' laserscanner_haw.launch & roslaunch '$HELPER_PKG' laserscan_filter.launch'
alias lsr='roslaunch '$HELPER_PKG' laserscanner_hokuyo_haw_mounted_on_extension_downright.launch'
alias cam='roslaunch camera1394 haw_cam.launch'
alias kinect='roslaunch '$HELPER_PKG' openni_fuerte_with_calibration_haw.launch'
alias cm='roslaunch '$HELPER_PKG' computer_monitor.launch'


# helper nodes
alias jms='rosrun joint_motion_service joint_motion_service'
alias is='roslaunch image_shrinker image_shrinker.launch'
alias kmf='roslaunch kinect_movement_filter kinect_movement_filter_haw.launch'
alias diag_agg='rosrun diagnostic_aggregator aggregator_node'


# computing nodes
alias gmapp='roslaunch '$HELPER_PKG' slam_gmapping_haw.launch'
alias gmapp_rear='roslaunch '$HELPER_PKG' slam_gmapping_haw_hokuyo_rear.launch'
alias mb='roslaunch '$HELPER_PKG' move_base_haw.launch'
alias mbnew='roslaunch scitos_2dnav move_base.launch'

alias octomap='roslaunch '$HELPER_PKG' octomap_mapping_haw_no_motion.launch'
alias collmap='roslaunch '$HELPER_PKG' collider_haw_no_motion.launch'
alias armnav='roslaunch scitos_haw_schunk_arm_navigation scitos_haw_schunk_arm_navigation_collision_map.launch'
alias warehouse='roslaunch scitos_haw_schunk_arm_navigation planning_scene_warehouse_viewer_scitos_haw_schunk_real_wo_rviz.launch'

alias telearm='rosrun teleop_arm_controller teleop_arm_controller'

alias map_server='rosrun map_server map_server ~/ros_workspace/recordings/Flur7_2013-02-04.yaml'
alias amcl='rosrun amcl amcl'


# interaction nodes
alias sgui='roslaunch schunk_gui start_gui_haw.launch'
alias rviz='rosrun rviz rviz'
alias rviz_felix='rviz --display-config $(rospack find '$HELPER_PKG')/config/rviz_felix.rviz'
alias rviz_felix_vcg='rviz --display-config $(rospack find '$HELPER_PKG')/config/rviz_felix.vcg'
alias smach_viewer='rosrun smach_viewer smach_viewer.py'

alias camera_raw_compressed='rosrun image_view image_view image:=/camera/image_raw compressed'
alias camera_small_compressed='rosrun image_view image_view image:=/camera/image_small compressed'
alias camera_small_theora='rosrun image_view image_view image:=/camera/image_small theora'
alias view_kinect_compressed='rosrun image_view image_view image:=/kinect1/rgb/image_color compressed'

alias tf='cd /var/tmp && rosrun tf view_frames && evince frames.pdf &'

alias kr='rosrun teleop_twist_keyboard teleop_twist_keyboard.py'
alias kra='rosrun teleop_twist_keyboard teleop_twist_keyboard_arm_cam.py'
alias ps3_teleop='roslaunch teleop_ps3 teleop_ps3.launch'
alias ps3_bt='pgrep ps3joy.py > /dev/null || sudo /opt/ros/groovy/lib/ps3joy/ps3joy.py --inactivity-timeout=300' # ignores multiple starts
alias ps3='ps3_bt & ps3_teleop & telearm'

alias jsp='roslaunch metralabs_ros scitos_haw_schunk_mockup.launch'

# small tools
alias rka='rosnode list; read -p "Kill all nodes? Press Enter or ^C:"; rosnode kill -a; rosnode list'
alias rnl='rosnode list'
alias rtl='rostopic list'

alias joint_states='rostopic echo -n 1 /joint_states'
alias joint_move_position='rostopic pub -1 /schunk/move_position metralabs_msgs/IDAndFloat -- '
alias joint_set_velocity='rostopic pub -1 /schunk/set_velocity metralabs_msgs/IDAndFloat -- '
alias joint_set_acceleration='rostopic pub -1 /schunk/set_acceleration metralabs_msgs/IDAndFloat -- '
alias joint_ref_gripper='rostopic pub -1 /schunk/ref std_msgs/Int8 5'

alias bumper_reset='rostopic pub -1 /bumper_reset std_msgs/Empty'
alias move_base_cancel_all='rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID "{stamp: { secs: 0 , nsecs: 0 } , id: ''}"'
