search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=rm_65.srdf
robot_name_in_srdf=rm_65
moveit_config_pkg=rm_65_moveit_config
robot_name=rm_65
planning_group_name=arm
ikfast_plugin_pkg=rm_65_arm_ikfast_plugin
base_link_name=base_link
eef_link_name=Link6
ikfast_output_path=/home/ydt/rwm_moveit/src/rm_robot/rm_65_arm_ikfast_plugin/rm_65_arm_ikfast_plugin/src/rm_65_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
