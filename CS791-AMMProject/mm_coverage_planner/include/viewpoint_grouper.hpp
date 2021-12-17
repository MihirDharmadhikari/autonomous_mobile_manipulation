#pragma once

// #include "utils/utils.hpp"
#include "utils/data_loader.hpp"

class ViewpointGrouper 
{
private:
	ros::NodeHandle nh_;
	ros::Publisher indiv_group_pub_;
	ros::Publisher indiv_base_pub_;
	ros::Publisher cuboid_vis_pub_;
	ros::Publisher all_base_locations_pub_;
	ros::Publisher test_cuboid_points_pub_;

	std::shared_ptr<SharedData> shared_data_;
	std::vector<ReachableCuboid> reachable_space_;
	std::vector<Eigen::Vector3d> cuboid_orientation_centers_;
public:
	ViewpointGrouper(std::shared_ptr<SharedData> shared_data, const ros::NodeHandle &nh);
	bool formGroups();
	bool loadReachableSpace(const std::string &file_name);
	bool loadOnecuboid(const std::string &file_name);
	bool isBaseValid(const Eigen::Vector3d &pos);
	bool adjustBasePosition(Eigen::Vector3d &base_pos, tf::Transform &T_W2Cub, const Eigen::Vector3d &cuboid_size);

	bool testSingleGroup();
	void loadDummyReachableSpace();
};