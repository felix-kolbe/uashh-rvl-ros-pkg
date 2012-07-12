HELPER_PKG=uashh_rvl_helper_files

alias aliases_edit='rosed '$HELPER_PKG' aliases.bash'
alias aliases_reload='source `rospack find $HELPER_PKG`/aliases.bash'


# config adjustments
alias ros_what='echo ROS_MASTER_URI=$ROS_MASTER_URI; echo ROS_ROOT=$ROS_ROOT; echo ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH'
alias ros_scitos='export ROS_MASTER_URI=http://scitos_w:11311'
alias ros_local='export ROS_MASTER_URI=http://localhost:11311'


# hardware nodes
alias ml='roslaunch metralabs_ros scitos_haw_only_start.launch'
alias mla='roslaunch metralabs_ros scitos_haw_schunk_start.launch'
alias rs='roslaunch rotoscan_node laserscanner_w_tf.launch'
alias cam='roslaunch camera1394 haw_cam.launch'
alias kinect='roslaunch '$HELPER_PKG' openni_fuerte_with_calibration_haw.launch'


# helper nodes
alias jms='rosrun joint_motion_service joint_motion_service'
alias is='roslaunch image_shrinker image_shrinker.launch'


# computing nodes
alias gmapp='roslaunch '$HELPER_PKG' slam_gmapping_haw.launch'
alias mb='roslaunch '$HELPER_PKG' move_base_haw.launch'
alias mbnew='roslaunch scitos_2dnav move_base.launch'


# interaction nodes
alias sgui='roslaunch schunk_gui start_gui_haw.launch'
alias rviz='rosrun rviz rviz'

alias camera_raw_compressed='rosrun image_view image_view image:=/camera/image_raw compressed'
alias camera_small_compressed='rosrun image_view image_view image:=/camera/image_small compressed'
alias camera_small_theora='rosrun image_view image_view image:=/camera/image_small theora'

alias kr='rosrun teleop_twist_keyboard teleop_twist_keyboard.py'
alias kra='rosrun teleop_twist_keyboard teleop_twist_keyboard_arm_cam.py'


# small tools
alias rka='rosnode list; rosnode kill -a; rosnode list'
alias rnl='rosnode list'

alias joint_states='rostopic echo -n 1 /schunk/position/joint_states'
alias joint_movePos='rostopic pub -1 /movePosition metralabs_ros/idAndFloat -- '
alias joint_targetVel='rostopic pub -1 /targetVelocity metralabs_ros/idAndFloat -- '
alias joint_targetAcc='rostopic pub -1 /targetAcceleration metralabs_ros/idAndFloat -- '
alias joint_ref_gripper='rostopic pub -1 /ref std_msgs/Int8 5'
