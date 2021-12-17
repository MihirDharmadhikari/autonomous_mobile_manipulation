#include "utils/data_loader.hpp"

DataLoader::DataLoader(std::shared_ptr<SharedData> shared_data, const ros::NodeHandle& nh)
		:nh_(nh), shared_data_(shared_data) 
{
	viewpoint_vis_pub_ = nh_.advertise<geometry_msgs::PoseArray>("viewpoint_vis", 10, true);
	test_cub_pub_ = nh_.advertise<geometry_msgs::PoseArray>("test_cuboid_outliers", 10, true);
	occ_pts_pub_ = nh_.advertise<geometry_msgs::PoseArray>("occ_pts", 10, true);
	test_cub_template_pub_ = nh_.advertise<geometry_msgs::PoseArray>("test_cuboid_templates", 10, true);
}

bool DataLoader::loadViewPoints(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << file_path << std::endl;
    return false;
  }

  std::string line;
	int id_counter = 0;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string vector, value;

		// Viewpoint
		std::getline(iss, vector, ':');
		std::istringstream vp_iss(vector);
		Eigen::Vector3d vp;
		for(int i=0;i<3;++i) {
			std::getline(vp_iss, value, ',');
			vp(i) = std::stod(value);
		}

		// Neighbor
		std::getline(iss, vector, ':');
		std::istringstream dir_iss(vector);
		Eigen::Vector3d dir;
		for(int i=0;i<3;++i) {
			std::getline(dir_iss, value, ',');
			dir(i) = std::stod(value);
		}
		ViewPoint vp_data;
		vp_data.id = id_counter;
		vp_data.center = vp;
		vp_data.direction = dir;
		vp_data.computePose();
		shared_data_->viewpoints_.push_back(vp_data);
		shared_data_->viewpoint_id_map_[id_counter] = vp_data;
		++id_counter;
  }

	std::cout << "Loaded " << shared_data_->viewpoints_.size() << " viewpoints." << std::endl;
	for(int i=0;i<shared_data_->viewpoints_.size();++i) {
		std::cout << shared_data_->viewpoints_[i].id << ": " << shared_data_->viewpoints_[i].center.transpose() << " " << shared_data_->viewpoints_[i].direction.transpose() << std::endl;
	}

  file.close();

  // Populate occupied points
  for(auto &vp : shared_data_->viewpoints_) {
	Eigen::Vector3d pt = vp.center;
	pt(2) = 0.0;
	shared_data_->occupied_points_.push_back(pt);
  }
  return true;
}

bool DataLoader::loadFullData(const std::string &file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << file_path << std::endl;
    return false;
  }

	std::string line;
	Eigen::Vector3d boat_centroid = Eigen::Vector3d::Zero();
	int counter = 0;
  while (std::getline(file, line)) {
    // Id
		std::istringstream iss(line);
		std::string id_str;
		std::string neighbors_str;
		std::string center_str;
		std::string direction_str;

		std::getline(iss, id_str, ':');
		int id = std::stoi(id_str);
		ViewPoint vp;
		vp.id = id;

		// Center
		std::getline(iss, center_str, ':');
		std::istringstream center_iss(center_str);
		Eigen::Vector3d center;
		for(int i=0;i<3;++i) {
			std::string value;
			std::getline(center_iss, value, ',');
			center(i) = std::stod(value);
		}
		vp.center = center;

		// Direction
		std::getline(iss, direction_str, ':');
		std::istringstream direction_iss(direction_str);
		Eigen::Vector3d direction;
		for(int i=0;i<3;++i) {
			std::string value;
			std::getline(direction_iss, value, ',');
			direction(i) = std::stod(value);
		}
		vp.direction = direction;

		vp.center -= 0.2 * vp.direction;

		boat_centroid += vp.center;
		++counter;

		vp.computePose();
	
		// z-axis limitation (transfer z values over 0.6m) is added for testing purposes
		//std::cout<<"shared_data_->viewpoints_[i].pose.position.z"<<typeid((float)vp.center.z()).name()<<"\n"<<std::endl;
		//std::cout<<"shared_data_->viewpoints_.size(): "<<shared_data_->viewpoints_.size()<<std::endl;
		// for(int i=0;i<vp.center.size();++i)
		// {
		// 	if ((float)(vp.center.z()) > 0.6f)
		// 	{
		// 		shared_data_->viewpoints_.push_back(vp);
		// 		shared_data_->viewpoint_id_map_[id] = vp;
		// 	}
		// }

		Eigen::Vector3d pt = vp.center + 0.3 * vp.direction;
		pt(2) = 0.0;
		shared_data_->occupied_points_.push_back(pt);

		if (vp.center.z() > 0.8)
		{
			continue;
		}

		shared_data_->viewpoints_.push_back(vp);
		shared_data_->viewpoint_id_map_[id] = vp;

		// Neighbors
		std::getline(iss, neighbors_str, ':');
		std::istringstream neighbors_iss(neighbors_str);
		while(std::getline(neighbors_iss, id_str, ',')) {
			//if ((float)(vp.center.z()) > 0.6f)
			//{
			shared_data_->viewpoint_neighbors_[id].push_back(std::stoi(id_str));
			//}
		}

	}
	std::cout << "Loaded " << shared_data_->viewpoints_.size() << " viewpoints." << std::endl;
	for(int i=0;i<shared_data_->viewpoints_.size();++i) {
		std::cout << shared_data_->viewpoints_[i].id << ": " << shared_data_->viewpoints_[i].center.transpose() << " " << shared_data_->viewpoints_[i].direction.transpose() << std::endl;
	}

	boat_centroid /= counter;
	boat_centroid.z() = 0.0;
	shared_data_->boat_centroid_ = boat_centroid;

	std::cout << "Viewpoint neighbors: " << std::endl;
	for(int i=0;i<shared_data_->viewpoints_.size();++i) {
		std::cout << shared_data_->viewpoints_[i].id << ": ";
		for(int j=0;j<shared_data_->viewpoint_neighbors_[i].size();++j) {
			std::cout << shared_data_->viewpoint_neighbors_[i][j] << " ";
		}
		std::cout << std::endl;
	}

	file.close();


	geometry_msgs::PoseArray occupied_points_msg;
	occupied_points_msg.header.frame_id = "map";
	for(auto &vp : shared_data_->occupied_points_) {
		geometry_msgs::Pose ps;
		ps.position.x = vp(0);
		ps.position.y = vp(1);
		ps.position.z = vp(2);
		ps.orientation.w = 1.0;
		occupied_points_msg.poses.push_back(ps);
	}
	occ_pts_pub_.publish(occupied_points_msg);

	return true;
}

void DataLoader::visualizeViewpoints()
{
	geometry_msgs::PoseArray pose_array;
	pose_array.header.frame_id = shared_data_->world_frame_;
	for(int i=0;i<shared_data_->viewpoints_.size();++i)
	{
		geometry_msgs::Pose p = shared_data_->viewpoints_[i].pose;
		pose_array.poses.push_back(p);
	}

	viewpoint_vis_pub_.publish(pose_array);
}

bool DataLoader::testSubgroups()
{
	std::vector<Eigen::Vector3d> orientation_centers;
	orientation_centers.push_back(Eigen::Vector3d(2.0, 0.0, 0.0));
	orientation_centers.push_back(Eigen::Vector3d(3.0, 0.0, 0.0));
	orientation_centers.push_back(Eigen::Vector3d(4.0, 0.0, 0.0));
	orientation_centers.push_back(Eigen::Vector3d(4.75, 0.0, 0.0));

	std::vector<ReachableCuboid> dummy_cuboids;
	ReachableCuboid c1;
	c1.center = Eigen::Vector3d(0.0, 0.0, 0.0);
	c1.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c1.mean_direction = Eigen::Vector3d(1.0, 0.0, 0.0);
	dummy_cuboids.push_back(c1);

	ReachableCuboid c2;
	c2.center = Eigen::Vector3d(0.0, 0.0, 0.0);
	c2.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c2.mean_direction = Eigen::Vector3d(0.78, 0.0, -0.78);
	dummy_cuboids.push_back(c2);

	ReachableCuboid c3;
	c3.center = Eigen::Vector3d(0.0, 0.0, 0.0);
	c3.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c3.mean_direction = Eigen::Vector3d(0.0, 0.0, -1.0);
	dummy_cuboids.push_back(c3);

	ReachableCuboid c4;
	c4.center = Eigen::Vector3d(0.0, 0.0, 0.0);
	c4.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c4.mean_direction = Eigen::Vector3d(0.78, 0.0, 0.78);
	dummy_cuboids.push_back(c4);

	geometry_msgs::PoseArray pa_templates;
	pa_templates.header.frame_id = shared_data_->world_frame_;
	for(int i=0; i<dummy_cuboids.size(); ++i)
	{
		geometry_msgs::Pose p = eigen2Pose(dummy_cuboids[i].center, dummy_cuboids[i].mean_direction);
		pa_templates.poses.push_back(p);
	}
	test_cub_template_pub_.publish(pa_templates);
	
	geometry_msgs::PoseArray pa;
	pa.header.frame_id = shared_data_->world_frame_;

	int num_outliers = 0;
	for(auto &vp : shared_data_->viewpoints_)
	{
		int i=0;
		double yaw = std::atan2(vp.direction.y(), vp.direction.x());
		for(i=0;i<dummy_cuboids.size();++i)
		{
			ReachableCuboid c = dummy_cuboids[i];
			if(c.isAligned(vp.direction, yaw))
			{
				break;
			}
		}
		if(i>=dummy_cuboids.size())
		{
			std::cout << "Viewpoint " << vp.id << " is not aligned with any cuboid." << std::endl;
			pa.poses.push_back(vp.pose);
			num_outliers++;
		}
		test_cub_pub_.publish(pa);
	}

	return true;
}