HELPER_PKG=uashh_rvl_helper_files

alias aliases_edit='rosed '$HELPER_PKG' aliases.bash'
alias aliases_reload='source `rospack find $HELPER_PKG`/aliases.bash'


# config adjustments
alias ros_what='echo ROS_MASTER_URI=$ROS_MASTER_URI; echo ROS_ROOT=$ROS_ROOT; echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH'
alias ros_scitos='export ROS_MASTER_URI=http://scitos_w:11311'
alias ros_local='export ROS_MASTER_URI=http://localhost:11311'


# all
alias uashh_all='roslaunch '$HELPER_PKG' uashh_all.launch'


# hardware nodes
alias ml='roslaunch metralabs_ros scitos_haw_only_start.launch'
alias mla='roslaunch metralabs_ros scitos_haw_schunk_start.launch'
alias rs='roslaunch rotoscan_node laserscanner_w_tf.launch'
alias cam='roslaunch camera1394 haw_cam.launch'
alias kinect='roslaunch '$HELPER_PKG' openni_fuerte_with_calibration_haw.launch'


# helper nodes
alias jms='rosrun joint_motion_service joint_motion_service'
alias is='roslaunch image_shrinker image_shrinker.launch'
alias kmf='roslaunch kinect_movement_filter kinect_movement_filter_haw.launch'

# computing nodes
alias gmapp='roslaunch '$HELPER_PKG' slam_gmapping_haw.launch'
alias mb='roslaunch '$HELPER_PKG' move_base_haw.launch'
alias mbnew='roslaunch scitos_2dnav move_base.launch'

alias octomap='roslaunch '$HELPER_PKG' octomap_mapping_haw_no_motion.launch'
alias collmap='roslaunch '$HELPER_PKG' collider_haw.launch'
alias armnav='roslaunch scitos_haw_schunk_arm_navigation scitos_haw_schunk_arm_navigation_collision_map.launch'
alias warehouse='roslaunch scitos_haw_schunk_arm_navigation planning_scene_warehouse_viewer_scitos_haw_schunk_real_wo_rviz.launch'

alias telearm='rosrun teleop_arm_controller teleop_arm_controller'

# interaction nodes
alias sgui='roslaunch schunk_gui start_gui_haw.launch'
alias rviz='rosrun rviz rviz'
alias rosgui='rosrun rqt_gui rqt_gui'

alias camera_raw_compressed='rosrun image_view image_view image:=/camera/image_raw compressed'
alias camera_small_compressed='rosrun image_view image_view image:=/camera/image_small compressed'
alias camera_small_theora='rosrun image_view image_view image:=/camera/image_small theora'
alias view_kinect_compressed='rosrun image_view image_view image:=/kinect1/rgb/image_color compressed'

alias kr='rosrun teleop_twist_keyboard teleop_twist_keyboard.py'
alias kra='rosrun teleop_twist_keyboard teleop_twist_keyboard_arm_cam.py'
alias ps3='roslaunch teleop_ps3 teleop_ps3.launch'
alias ps3_bt='pgrep ps3joy.py > /dev/null || sudo /opt/ros/fuerte/stacks/joystick_drivers/ps3joy/ps3joy.py' # don't start it twice
alias ps3_full='ps3_bt & ps3 & telearm'

# small tools
#alias rka='rosnode list; rosnode kill -a; rosnode list'
alias rnl='rosnode list'
alias rtl='rostopic list'

alias joint_states='rostopic echo -n 1 /schunk/position/joint_states'
alias joint_movePos='rostopic pub -1 /movePosition metralabs_ros/idAndFloat -- '
alias joint_targetVel='rostopic pub -1 /targetVelocity metralabs_ros/idAndFloat -- '
alias joint_targetAcc='rostopic pub -1 /targetAcceleration metralabs_ros/idAndFloat -- '
alias joint_ref_gripper='rostopic pub -1 /ref std_msgs/Int8 5'
