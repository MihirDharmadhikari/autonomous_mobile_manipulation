
#!/bin/sh

xterm   -e  " roslaunch robowork_gazebo bvr_SIM_playpen.launch" &
sleep 10
xterm   -e  " ROS_NAMESPACE="bvr_SIM" roslaunch robowork_moveit_config robowork_moveit_planning_execution.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM" &
sleep 10
xterm   -e  " roslaunch robowork_moveit_config moveit_rviz.launch " &
sleep 10
xterm   -e   " ROS_NAMESPACE="bvr_SIM" roslaunch robowork_ur_launch ur5e_compliance.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM " &
sleep 10
xterm   -e  " rosservice call /compliance_controller/toggle_compliance "{}" " &
sleep 10
xterm   -e   " ROS_NAMESPACE="bvr_SIM" roslaunch mm_coverage_planner move_group_interface_vBoat.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM sim_suffix:=_SIM "
#xterm  -e  " ROS_NAMESPACE="bvr_SIM" roslaunch robowork_ur_launch ur5e_compliance.launch robot_namespace:=bvr_SIM arm_namespace:=main_arm_SIM" &
#sleep 10
#xterm  -e  " rosservice call /compliance_controller/toggle_compliance "{}" "
