#include "viewpoint_grouper.hpp"

ViewpointGrouper::ViewpointGrouper(std::shared_ptr<SharedData> shared_data, const ros::NodeHandle &nh)
	: shared_data_(shared_data), nh_(nh)
{
  indiv_group_pub_ = nh_.advertise<geometry_msgs::PoseArray>("indiv_group", 10, true);
	indiv_base_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("indiv_base", 10, true);
	cuboid_vis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("cuboid_vis", 10, true);
	all_base_locations_pub_ = nh_.advertise<geometry_msgs::PoseArray>("all_base_locations", 10, true);
	test_cuboid_points_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("test_cuboid_edge", 10, true);
}

bool ViewpointGrouper::loadOnecuboid(const std::string &file_name)
{
	std::ifstream ifs(file_name);
	if (!ifs.is_open())
	{
		ROS_ERROR("Failed to open file %s", file_name.c_str());
		return false;
	}
	bool reading_points = false;
	bool reading_cuboid = false;
	std::string line;
	std::vector<Eigen::Vector3d> points;
	std::vector<Eigen::Vector3d> cuboid_vertices;
	while (std::getline(ifs, line))
	{
		if(line == "CUBOID")
		{
			std::cout << "Reading cuboid" << std::endl;
			reading_cuboid = true;
			reading_points = false;
			continue;
		}
		if(line == "POINTS")
		{
			std::cout << "Reading points" << std::endl;
			reading_points = true;
			continue;
		}

		if(reading_points)
		{
			std::istringstream iss(line);
			double x, y, z;
			iss >> x >> y >> z;
			Eigen::Vector3d point(x, y, z);
			std::cout << point.transpose() << std::endl;
			points.push_back(point);
		}

		if(reading_cuboid)
		{
			std::istringstream iss(line);
			double x, y, z;
			iss >> x >> y >> z;
			Eigen::Vector3d point(x, y, z);
			std::cout << point.transpose() << std::endl;
			cuboid_vertices.push_back(point);
		}
	}

	visualization_msgs::Marker cuboid_points;
	cuboid_points.header.frame_id = shared_data_->world_frame_;
	cuboid_points.type = visualization_msgs::Marker::SPHERE_LIST;
	cuboid_points.action = visualization_msgs::Marker::ADD;
	cuboid_points.id = 0;
	cuboid_points.scale.x = 0.7;
	cuboid_points.scale.y = 0.7;
	cuboid_points.scale.z = 0.7;
  cuboid_points.color.r = 200.0 / 255.0;
  cuboid_points.color.g = 100.0 / 255.0;
  cuboid_points.color.b = 0.0;
  cuboid_points.color.a = 1.0;

	cuboid_points.points.clear();

	for(int i=0;i<points.size();++i)
	{
		geometry_msgs::Point p1;
		p1.x = points[i].x();
		p1.y = points[i].y();
		p1.z = points[i].z();

		cuboid_points.points.push_back(p1);
	}

	visualization_msgs::MarkerArray edge_array;
	edge_array.markers.push_back(cuboid_points);
	test_cuboid_points_pub_.publish(edge_array);

	ReachableCuboid cuboid;

	Eigen::Vector3d center = cuboid_vertices[0];
	Eigen::Vector3d left = cuboid_vertices[2];
	Eigen::Vector3d up = cuboid_vertices[1];
	Eigen::Vector3d front = cuboid_vertices[4];

	Eigen::Vector3d dir1 = front - center;
	Eigen::Vector3d dir2 = left - center;
	Eigen::Vector3d dir3 = up - center;

	std::cout << "dir1: " << dir1.transpose() << std::endl;
	std::cout << "dir2: " << dir2.transpose() << std::endl;
	std::cout << "dir3: " << dir3.transpose() << std::endl;

	Eigen::Vector3d rotations;
	rotations(2) = atan2(dir1(1), dir1(0));       // Yaw
	rotations(1) = asin(-dir1(2) / dir1.norm());  // Pitch
	rotations(0) = asin(dir2(2) / dir2.norm());   // Roll

	cuboid.center = Eigen::Vector3d::Zero();
	for(int i=0;i<cuboid_vertices.size();++i)
	{
		cuboid.center += cuboid_vertices[i];
	}
	cuboid.center /= cuboid_vertices.size();

	cuboid.size = Eigen::Vector3d(dir1.norm(), dir2.norm(), dir3.norm());

	cuboid.rotations = rotations;

	reachable_space_.push_back(cuboid);

	visualization_msgs::Marker cuboid_vis = cuboid.getMarker(Eigen::Vector3d(0, 0, 0), 0.0, 0, shared_data_->world_frame_);
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.push_back(cuboid_vis);
	cuboid_vis_pub_.publish(marker_array);

	return true;
}

bool ViewpointGrouper::loadReachableSpace(const std::string &file_name)
{
	return true;
}

void ViewpointGrouper::loadDummyReachableSpace()
{
	reachable_space_.clear();
	ReachableCuboid c1;
	// c1.center = Eigen::Vector3d(0.7, 0.2, 0.6);
	c1.center = Eigen::Vector3d(0.65, 0.0, 0.6);
	c1.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c1.mean_direction = Eigen::Vector3d(0.0, 0.0, -1.0);
	// c1.size = Eigen::Vector3d(0.75, 1.0, 1.0);
	c1.size = Eigen::Vector3d(0.5, 0.75, 0.75);
	reachable_space_.push_back(c1);

	ReachableCuboid c2;
	c2.center = Eigen::Vector3d(0.6, 0.0, 0.6);
	// c1.center = Eigen::Vector3d(0.65, 0.0, 0.6);
	c2.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c2.mean_direction = Eigen::Vector3d(0.78, 0.0, -0.78);
	c2.size = Eigen::Vector3d(0.75, 0.8, 1.0);
	// c1.size = Eigen::Vector3d(0.75, 1.0, 1.0);
	reachable_space_.push_back(c2);

	ReachableCuboid c3;
	c3.center = Eigen::Vector3d(0.8, 0.0, 0.5);
	// c3.center = Eigen::Vector3d(0.6, 0.0, 0.5);
	c3.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c3.mean_direction = Eigen::Vector3d(1.0, 0.0, 0.0);
	c3.size = Eigen::Vector3d(0.5, 0.7, 0.7);
	// c3.size = Eigen::Vector3d(0.3, 0.5, 0.7);
	reachable_space_.push_back(c3);

	ReachableCuboid c4;
	c4.center = Eigen::Vector3d(0.8, 0.0, 0.5);
	c4.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c4.mean_direction = Eigen::Vector3d(0.78, 0.0, 0.78);
	c4.size = Eigen::Vector3d(0.5, 0.5, 0.7);
	reachable_space_.push_back(c4);
}

bool ViewpointGrouper::formGroups()
{
	/*
	Create list of all viewpoints
	while list not empty:
		sample a viewpoint from the list
		classify into correct cuboid and orient it 
		Calculate base location
		Check if base location is valid
		if yes:
			Create a new group
			Find all vps lying in this cuboid and aligned to the mean direction
			Add to the subgroup of this cuboid
			Remove those vps from the list
			for each cuboid reachable from this base location:
				Find vps lying in this cuboid and aligned to the mean direction
				Add to the subgroup of this cuboid
				Remove those vps from the list
			add subgroups to the group
			add group to the list of groups
	*/

	visualization_msgs::MarkerArray reachable_space_markers;

	std::vector<ViewPoint> tmp_vps = shared_data_->viewpoints_;
	std::random_shuffle(tmp_vps.begin(), tmp_vps.end());
	std::list<ViewPoint> vps_list(tmp_vps.begin(), tmp_vps.end());

	int iter_conter = 0;
	int iter_limit_slack = 5;

	while(!vps_list.empty())
	{
		// To make sure that the loop ends at some point
		// Number of iterations in this loop should not be more than the number of viewpoints
		// The additional 'iter_limit_slack' iterations are just for safety
		++iter_conter;
		if(iter_conter >= shared_data_->viewpoints_.size() * 10)
			break;

		ViewPoint vp = vps_list.front();
		vps_list.pop_front();

		// Classify viewpoint into cuboid
		int space_id = -1;
		double yaw = std::atan2(vp.direction.y(), vp.direction.x());
		for(int i=0;i<reachable_space_.size();++i)
		{
			ReachableCuboid cuboid = reachable_space_[i];
			if(cuboid.isAligned(vp.direction, yaw))
			{
				space_id = i;
				break;
			}
		}
		if(space_id < 0)
		{
			vps_list.push_back(vp);
			continue;
		}

		tf::Transform T_W2Cub, T_B2Cub;
		Eigen::Matrix3d R_W2Cub;
		R_W2Cub = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
		Eigen::Quaterniond quat_W2Cub_eig(R_W2Cub);
		tf::Quaternion quat_W2Cub(quat_W2Cub_eig.x(), quat_W2Cub_eig.y(), quat_W2Cub_eig.z(), quat_W2Cub_eig.w());
		T_W2Cub.setRotation(quat_W2Cub);
		T_W2Cub.setOrigin(eigen2Tf(vp.center));

		Eigen::Matrix3d R_B2Cub;
		R_B2Cub = Eigen::AngleAxisd(reachable_space_[space_id].rotations.z(), Eigen::Vector3d::UnitZ());  // Here the Cub frame is z aligned with the world frame
		Eigen::Quaterniond quat_B2Cub_eig(R_B2Cub);
		tf::Quaternion quat_B2Cub(quat_B2Cub_eig.x(), quat_B2Cub_eig.y(), quat_B2Cub_eig.z(), quat_B2Cub_eig.w());
		T_B2Cub.setRotation(quat_B2Cub);
		T_B2Cub.setOrigin(eigen2Tf(reachable_space_[space_id].center));
		tf::Vector3 base_in_W_tf = T_W2Cub * T_B2Cub.inverse() * tf::Vector3(0.0, 0.0, 0.0);
		Eigen::Vector3d base_in_W = tf2Eigen(base_in_W_tf);

		std::cout << "Found base location: " << base_in_W.transpose() << std::endl;
		// Move the base to correct location
		if(!adjustBasePosition(base_in_W, T_W2Cub, reachable_space_[space_id].size))
		{
			std::cout << "Base adjustment failed" << std::endl;
			vps_list.push_back(vp);
			continue;
		}
		Eigen::Vector3d new_vp_in_B = tf2Eigen(T_B2Cub * T_W2Cub.inverse() * eigen2Tf(vp.center));
		if(!reachable_space_[space_id].isInside(new_vp_in_B))
		{
			std::cout << "Viewpoint is not inside the cuboid after base position correction" << std::endl;
			vps_list.push_back(vp);
			continue;
		}


		for(int i=0;i<reachable_space_.size();++i)
		{
			visualization_msgs::Marker cub_marker = reachable_space_[i].getMarker(base_in_W, yaw, i, shared_data_->world_frame_);
			reachable_space_markers.markers.push_back(cub_marker);
		}

		geometry_msgs::PoseStamped base_pose_W;
		base_pose_W.header.frame_id = shared_data_->world_frame_;
		base_pose_W.pose = eigen2Pose(base_in_W, yaw);
		indiv_base_pub_.publish(base_pose_W);

		ViewPointGroup current_group;
		double sp_yaw = yaw + M_PI/2.0;
		if(sp_yaw > M_PI)
			sp_yaw -= 2.0 * M_PI;
		else if(sp_yaw < -M_PI)
			sp_yaw += 2.0 * M_PI;
		// current_group.base_pose = eigen2Pose(base_in_W, sp_yaw);
		current_group.base_pose = eigen2Pose(base_in_W, yaw);

		std::vector<ViewPoint> first_subgroup;
		geometry_msgs::PoseArray subgroup_poses;
		subgroup_poses.header.frame_id = shared_data_->world_frame_;
		first_subgroup.push_back(vp);
		subgroup_poses.poses.push_back(vp.pose);

		std::list<ViewPoint> temp_vp_list = vps_list;
		while(!temp_vp_list.empty())
		{
			ViewPoint nvp = temp_vp_list.front();
			temp_vp_list.pop_front();

			Eigen::Vector3d center_B;
			center_B = tf2Eigen(T_B2Cub * T_W2Cub.inverse() * eigen2Tf(nvp.center));
			if(reachable_space_[space_id].isInside(center_B))
			{
				if(reachable_space_[space_id].isAligned(nvp.direction, yaw))
				{
					first_subgroup.push_back(nvp);
					vps_list.remove(nvp);
					temp_vp_list.remove(nvp);
					subgroup_poses.poses.push_back(nvp.pose);
				}
			}
		}
		current_group.viewpoint_subgroups.push_back(first_subgroup);
		// Try other templates for this base location
		temp_vp_list = vps_list;
		for(int i=0;i<reachable_space_.size();++i)
		{
			if(i == space_id)
			{
				continue;
			}
			std::vector<ViewPoint> next_subgroup;
			while(!temp_vp_list.empty())
			{
				ViewPoint nvp = temp_vp_list.front();
				temp_vp_list.pop_front();

				Eigen::Vector3d center_B;
				center_B = tf2Eigen(T_B2Cub * T_W2Cub.inverse() * eigen2Tf(nvp.center));
				if(reachable_space_[i].isInside(center_B))
				{
					if(reachable_space_[i].isAligned(nvp.direction, yaw))
					{
						next_subgroup.push_back(nvp);
						vps_list.remove(nvp);
						temp_vp_list.remove(nvp);
						subgroup_poses.poses.push_back(nvp.pose);
					}
				}
			}
			if(!next_subgroup.empty())
			{
				current_group.viewpoint_subgroups.push_back(next_subgroup);
			}
		}
		shared_data_->viewpoint_groups_.push_back(current_group);
		indiv_group_pub_.publish(subgroup_poses);
		cuboid_vis_pub_.publish(reachable_space_markers);
		std::cout << "Iteration: " << iter_conter << " Remaining viewpoints: " << vps_list.size() << std::endl;
		// break;
		// ros::Rate(5).sleep();
	}

	geometry_msgs::PoseArray all_base_locations;
	all_base_locations.header.frame_id = shared_data_->world_frame_;
	for(auto &g : shared_data_->viewpoint_groups_)
	{
		all_base_locations.poses.push_back(g.base_pose);
	}
	all_base_locations_pub_.publish(all_base_locations);

	if(iter_conter >= shared_data_->viewpoints_.size() + iter_limit_slack)
	{
		std::cout << "WARN: Loop exited before classifying all viewpoints. Some viewpoints are not classified into groups: " << vps_list.size() << std::endl;
		return false;
	}
	else
		return true;

}

bool ViewpointGrouper::isBaseValid(const Eigen::Vector3d &pos)
{
	// std::vector<ViewPoint> neighbor_points;
	// for(auto &nvp : shared_data_->viewpoints_)
	// {
	// 	if((nvp.center - pos).norm() <= shared_data_->robot_radius_)
	// 	{
	// 		// Viewpoint is within robot radius
	// 	}
	// }
	// TODO
	return true;
}

bool ViewpointGrouper::adjustBasePosition(Eigen::Vector3d &base_pos, tf::Transform &T_W2Cub, const Eigen::Vector3d &cuboid_size)
{
	double delta_z = base_pos.z() - shared_data_->base_z_;
	if(std::abs(delta_z) > cuboid_size.z()/2.0)
	{
		return false;
	}
	else
	{
		T_W2Cub.setOrigin(eigen2Tf(tf2Eigen(T_W2Cub.getOrigin()) - Eigen::Vector3d(0.0, 0.0, delta_z)));
		base_pos.z() = shared_data_->base_z_;
	}

	Eigen::Vector3d closest_occ_pt;
	std::vector<Eigen::Vector3d> close_by_pts;
	double min_obstacle_dist = std::numeric_limits<double>::max();
	for(int i=0;i<shared_data_->occupied_points_.size();++i)
	{
		double dist = (shared_data_->occupied_points_[i] - base_pos).head(2).norm();
		if(dist < 2.0 * shared_data_->reduced_robot_radius_)
		{
			close_by_pts.push_back(shared_data_->occupied_points_[i]);
		}
		if(dist < min_obstacle_dist)
		{
			min_obstacle_dist = dist;
			closest_occ_pt = shared_data_->occupied_points_[i];
		}
	}

	std::cout << "Min obstacle dist: " << min_obstacle_dist << std::endl;

	tf::Quaternion q_W2Cub = T_W2Cub.getRotation();
	tf::Matrix3x3 R_W2Cub(q_W2Cub);
	double roll, pitch, yaw;
	R_W2Cub.getRPY(roll, pitch, yaw);
	
	if(min_obstacle_dist > shared_data_->reduced_robot_radius_)
	{
		return true;
	}
	else
	{
		// Base is too close to an obstacle move it back
		std::cout << "Base is too close to an obstacle. " << min_obstacle_dist << " Moving back" << std::endl;
		if(min_obstacle_dist > 2.0 * (cuboid_size.x()/2.0))
		{
			std::cout << "Can't adjust base" << std::endl;
			return false;
		}
		std::cout << "Old base: " << base_pos.transpose() << std::endl;
		Eigen::Vector3d delta_pos = Eigen::Vector3d(-std::cos(yaw), -std::sin(yaw), 0.0);
		int i = 0;
		for(i=0;i<10;++i)
		{
			min_obstacle_dist = std::numeric_limits<double>::max();
			Eigen::Vector3d new_base_pos = base_pos + shared_data_->reduced_robot_radius_/5.0 * delta_pos.normalized();
			base_pos = new_base_pos;
			std::cout << "Updated base: " << base_pos.transpose() << std::endl;
			for(int i=0;i<shared_data_->occupied_points_.size();++i)
			{
				double dist = (shared_data_->occupied_points_[i] - base_pos).head(2).norm();
				if(dist < min_obstacle_dist)
				{
					min_obstacle_dist = dist;
				}
			}
			std::cout << "New Min obstacle dist: " << min_obstacle_dist << std::endl;
			if(min_obstacle_dist > shared_data_->reduced_robot_radius_ * 1.2)
			{
				std::cout << "Base adjusted" << std::endl;
				break;
			}
		}
		if(i >= 10)
		{
			std::cout << "Can't adjust base" << std::endl;
			return false;
		}
		std::cout << "New base: " << base_pos.transpose() << std::endl;
		T_W2Cub.setOrigin(eigen2Tf(tf2Eigen(T_W2Cub.getOrigin()) + min_obstacle_dist * delta_pos.normalized()));
	}
	return true;
}

bool ViewpointGrouper::testSingleGroup()
{
	std::vector<ReachableCuboid> dummy_cuboids;
	ReachableCuboid c1;
	// c1.center = Eigen::Vector3d(0.5, 0.0, 0.6);
	c1.center = Eigen::Vector3d(0.5, 0.0, 0.5);
	c1.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c1.mean_direction = Eigen::Vector3d(0.0, 0.0, -1.0);
	// c1.size = Eigen::Vector3d(0.6, 0.7, 0.3);	
	c1.size = Eigen::Vector3d(0.5, 0.5, 0.7);
	dummy_cuboids.push_back(c1);

	ReachableCuboid c2;
	c2.center = Eigen::Vector3d(0.5, 0.0, 0.5);
	c2.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c2.mean_direction = Eigen::Vector3d(0.78, 0.0, -0.78);
	c2.size = Eigen::Vector3d(0.5, 0.5, 0.7);
	dummy_cuboids.push_back(c2);

	ReachableCuboid c3;
	// c3.center = Eigen::Vector3d(0.7, 0.0, 0.5);
	c3.center = Eigen::Vector3d(0.5, 0.0, 0.5);
	c3.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c3.mean_direction = Eigen::Vector3d(1.0, 0.0, 0.0);
	// c3.size = Eigen::Vector3d(0.3, 0.7, 0.7);
	c3.size = Eigen::Vector3d(0.5, 0.5, 0.7);
	dummy_cuboids.push_back(c3);

	ReachableCuboid c4;
	c4.center = Eigen::Vector3d(0.5, 0.0, 0.5);
	c4.rotations = Eigen::Vector3d(0.0, 0.0, 0.0);
	c4.mean_direction = Eigen::Vector3d(0.78, 0.0, 0.78);
	c4.size = Eigen::Vector3d(0.5, 0.5, 0.7);
	dummy_cuboids.push_back(c4);

	visualization_msgs::MarkerArray dummy_cuboids_markers;

	std::vector<ViewPoint> tmp_vps = shared_data_->viewpoints_;
	std::random_shuffle(tmp_vps.begin(), tmp_vps.end());
	std::list<ViewPoint> vps_list(tmp_vps.begin(), tmp_vps.end());

	int iter_conter = 0;
	int iter_limit_slack = 5;

	while(!vps_list.empty())
	{
		// To make sure that the loop ends at some point
		// Number of iterations in this loop should not be more than the number of viewpoints
		// The additional 'iter_limit_slack' iterations are just for safety
		++iter_conter;
		if(iter_conter >= shared_data_->viewpoints_.size() + iter_limit_slack)
			break;

		ViewPoint vp = vps_list.front();
		vps_list.pop_front();

		// Classify viewpoint into cuboid
		int space_id = -1;
		double yaw = std::atan2(vp.direction.y(), vp.direction.x());
		for(int i=0;i<dummy_cuboids.size();++i)
		{
			ReachableCuboid cuboid = dummy_cuboids[i];
			if(cuboid.isAligned(vp.direction, yaw))
			{
				space_id = i;
				break;
			}
		}
		if(space_id < 0)
		{
			vps_list.push_back(vp);
			continue;
		}

		tf::Transform T_W2Cub, T_B2Cub;
		Eigen::Matrix3d R_W2Cub;
		R_W2Cub = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
		Eigen::Quaterniond quat_W2Cub_eig(R_W2Cub);
		tf::Quaternion quat_W2Cub(quat_W2Cub_eig.x(), quat_W2Cub_eig.y(), quat_W2Cub_eig.z(), quat_W2Cub_eig.w());
		T_W2Cub.setRotation(quat_W2Cub);
		T_W2Cub.setOrigin(eigen2Tf(vp.center));

		Eigen::Matrix3d R_B2Cub;
		R_B2Cub = Eigen::AngleAxisd(dummy_cuboids[space_id].rotations.z(), Eigen::Vector3d::UnitZ());  // Here the Cub frame is z aligned with the world frame
		Eigen::Quaterniond quat_B2Cub_eig(R_B2Cub);
		tf::Quaternion quat_B2Cub(quat_B2Cub_eig.x(), quat_B2Cub_eig.y(), quat_B2Cub_eig.z(), quat_B2Cub_eig.w());
		T_B2Cub.setRotation(quat_B2Cub);
		T_B2Cub.setOrigin(eigen2Tf(dummy_cuboids[space_id].center));
		tf::Vector3 base_in_W_tf = T_W2Cub * T_B2Cub.inverse() * tf::Vector3(0.0, 0.0, 0.0);
		Eigen::Vector3d base_in_W = tf2Eigen(base_in_W_tf);

		std::cout << "Found base location: " << base_in_W.transpose() << std::endl;
		// Move the base to correct height
		double delta_z = base_in_W.z() - shared_data_->base_z_;
		if(std::abs(delta_z) > dummy_cuboids[space_id].size.z()/2.0)
		{
			std::cout << "Base is too high" << std::endl;
			vps_list.push_back(vp);
			continue;
		}
		else
		{
			base_in_W.z() -= delta_z;
			T_W2Cub.setOrigin(eigen2Tf(vp.center - Eigen::Vector3d(0.0, 0.0, delta_z)));
		}

		for(int i=0;i<dummy_cuboids.size();++i)
		{
			visualization_msgs::Marker cub_marker = dummy_cuboids[i].getMarker(base_in_W, yaw, i, shared_data_->world_frame_);
			dummy_cuboids_markers.markers.push_back(cub_marker);
		}

		geometry_msgs::PoseStamped base_pose_W;
		base_pose_W.header.frame_id = shared_data_->world_frame_;
		base_pose_W.pose = eigen2Pose(base_in_W, yaw);
		indiv_base_pub_.publish(base_pose_W);

		ViewPointGroup current_group;
		current_group.base_pose = eigen2Pose(base_in_W, yaw);

		std::vector<ViewPoint> subgroup;
		geometry_msgs::PoseArray subgroup_poses;
		subgroup_poses.header.frame_id = shared_data_->world_frame_;
		subgroup.push_back(vp);
		subgroup_poses.poses.push_back(vp.pose);

		std::list<ViewPoint> temp_vp_list = vps_list;
		while(!temp_vp_list.empty())
		{
			ViewPoint nvp = temp_vp_list.front();
			temp_vp_list.pop_front();

			Eigen::Vector3d center_B;
			center_B = tf2Eigen(T_B2Cub * T_W2Cub.inverse() * eigen2Tf(nvp.center));
			if(dummy_cuboids[space_id].isInside(center_B))
			{
				if(dummy_cuboids[space_id].isAligned(nvp.direction, yaw))
				{
					subgroup.push_back(nvp);
					vps_list.remove(nvp);
					temp_vp_list.remove(nvp);
					subgroup_poses.poses.push_back(nvp.pose);
				}
			}
		}
		// Try other templates for this base location
		temp_vp_list = vps_list;
		for(int i=0;i<dummy_cuboids.size();++i)
		{
			if(i == space_id)
			{
				continue;
			}
			while(!temp_vp_list.empty())
			{
				ViewPoint nvp = temp_vp_list.front();
				temp_vp_list.pop_front();

				Eigen::Vector3d center_B;
				center_B = tf2Eigen(T_B2Cub * T_W2Cub.inverse() * eigen2Tf(nvp.center));
				if(dummy_cuboids[i].isInside(center_B))
				{
					if(dummy_cuboids[i].isAligned(nvp.direction, yaw))
					{
						subgroup.push_back(nvp);
						vps_list.remove(nvp);
						temp_vp_list.remove(nvp);
						subgroup_poses.poses.push_back(nvp.pose);
					}
				}
			}
		}
		indiv_group_pub_.publish(subgroup_poses);
		cuboid_vis_pub_.publish(dummy_cuboids_markers);
		// break;
		std::cout << "Iteration: " << iter_conter << " Remaining viewpoints: " << vps_list.size() << std::endl;
		ros::Rate(2).sleep();
	}
	return true;
}