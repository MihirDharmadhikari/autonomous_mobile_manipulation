#include <iostream>
#include <vector>
#include <memory>
#include <cmath>
#include <fstream>
#include <list>
#include <unordered_map>

#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <move_base_msgs/MoveBaseAction.h>


struct ViewPoint
{
	int id;
	Eigen::Vector3d center;
	Eigen::Vector3d direction;
	geometry_msgs::Pose pose;

	void computePose();

	inline bool operator==(ViewPoint v)
	{
		if(this->id == v.id && (this->center - v.center).norm() <= 0.001 && (this->direction - v.direction).norm() <= 0.001)
			return true;
		else
			return false;
	}
};

struct ReachableCuboid
{
	Eigen::Vector3d size;
	Eigen::Vector3d rotations;
	Eigen::Vector3d center;
	Eigen::Vector3d mean_direction;
	double angle_threshold = M_PI/8;

	Eigen::Matrix3d getRotationMatrix() { return rot_B2Cub; }
	bool isInside(Eigen::Vector3d& pos);  // The point needs to be in the base frame
	bool isAligned(Eigen::Vector3d& direction);
	bool isAligned(Eigen::Vector3d& direction, double yaw);
	visualization_msgs::Marker getMarker(const Eigen::Vector3d& pos, const double& yaw, int color, std::string frame_id);

 private:
  Eigen::Matrix3d rot_B2Cub;
	tf::Transform T_B2Cub;
};

struct ViewPointGroup
{
	geometry_msgs::Pose base_pose;
	std::vector<std::vector<ViewPoint>> viewpoint_subgroups;
};

Eigen::Vector3d tf2Eigen(const tf::Vector3 &v_tf);
tf::Vector3 eigen2Tf(const Eigen::Vector3d &v);
geometry_msgs::Pose eigen2Pose(const Eigen::Vector3d &center, const Eigen::Vector3d &direction);
geometry_msgs::Pose eigen2Pose(const Eigen::Vector3d &center, const double yaw);

/*
This struct stores the information that will be shared among different objects
A shared pointer of this struct is created in the main file and passed to each object that needs it
The appropriate objects will populate the information for the other objects to use
Please check the DataLoader class and test_main.cpp for an example use of this struct
*/
struct SharedData 
{
	double mesh_inflation_distance_;
	double reduced_robot_radius_ = 0.4;
	std::string world_frame_ = "map";
	double base_z_ = 0.0;

	std::vector<ViewPoint> viewpoints_;  // In coordinate frame of the mesh
	std::map<int, std::vector<int>> viewpoint_neighbors_;  // VP id to neighbors
	std::map<int, ViewPoint> viewpoint_id_map_;  // VP id to VP
	std::vector<Eigen::Vector3d> occupied_points_;
	
	std::vector<ViewPointGroup> viewpoint_groups_;

	Eigen::Vector3d boat_centroid_;
};