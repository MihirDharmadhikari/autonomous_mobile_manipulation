#pragma once
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <math.h>
#include <typeinfo>
#include <fstream>
#include <limits.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include "viewpoint_grouper.hpp"

#include <moveit/move_group/node_name.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// The tf package provides an implementation of a TransformListener to help make the task of receiving transforms easier. 
// To use the TransformListener, we need to include the tf/transform_listener.h header file.
#include <tf/transform_listener.h>

#include <sensor_msgs/Joy.h>


class PathPlanner
{
private:
	ros::NodeHandle nh_;
	ros::Publisher base_loc_pub_;
	ros::Publisher bvr_goal_pub_;
    ros::Publisher cmd_vel_pub_;
	ros::Subscriber bvr_goal_stat_sub_;
    ros::Subscriber rc_sub_;

	std::shared_ptr<SharedData> shared_data_;
	uint8_t goalStatus;
	int base_location_counter_; //base_location_counter
    bool base_tsp_flag=0;
    bool arm_tsp_flag=0;
    int vp_counter_; // viewpoint counter
    int base_route_[50]; // base locations optimal route
    int arm_route_[100]; // viewpoint locations optimal route
    int unreachable_vps_counter_;
    int total_vps_counter_;
	void goalCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

    // moveit additions
    // ROS-related heavyweight setup
    void armInitialize();
    // execute planning attempt
    void armPlan();
    
    ros::NodeHandle private_nh_;
    const std::string world_link_;
    std::string plan_link_;
    std::string ee_link_;

    std::string PLANNING_GROUP_;

    std::map<std::string, tf::StampedTransform> object_map_;

    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::unique_ptr<tf::TransformListener> tf_listener_;

    const robot_state::JointModelGroup* joint_model_group_;

    tf::StampedTransform T_EE_W_;  // end-effector to world
    tf::Stamped<tf::Pose> T_EE_ref_W_{tf::Pose(), ros::Time(0), ""};  // end-effector-reference to world

public:
    PathPlanner(std::shared_ptr<SharedData> shared_data, const ros::NodeHandle &nh, const ros::NodeHandle &private_nh, std::string world_link);
    void baseOptPath();
    void armOptPath();
    void tsp(std::vector<std::vector<float>> tspCost);
	void visualizeBaseLocations();
};