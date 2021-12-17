#pragma once

#include <string>
#include <map>

#include "utils/utils.hpp"

class DataLoader
{
private:
	ros::NodeHandle nh_;

	ros::Publisher viewpoint_vis_pub_;
	ros::Publisher test_cub_pub_;
	ros::Publisher occ_pts_pub_;
	ros::Publisher test_cub_template_pub_;

	std::shared_ptr<SharedData> shared_data_;

public:
  DataLoader(std::shared_ptr<SharedData> shared_data, const ros::NodeHandle &nh);
	bool loadViewPoints(const std::string &file_name);
	bool loadFullData(const std::string &file_name);

	void visualizeViewpoints();
	bool testSubgroups();
};