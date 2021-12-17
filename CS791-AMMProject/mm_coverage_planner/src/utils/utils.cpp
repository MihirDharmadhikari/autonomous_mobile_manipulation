#include "utils/utils.hpp"

void ViewPoint::computePose()
{
	this->pose.position.x = this->center.x();
	this->pose.position.y = this->center.y();
	this->pose.position.z = this->center.z();

	double r = this->direction.head(2).norm();
	double phi = atan2(this->direction.z(), r);
	double theta = atan2(this->direction.y(), this->direction.x());

	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(-phi, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
	
	Eigen::Quaterniond quat(R);

	this->pose.orientation.x = quat.x();
	this->pose.orientation.y = quat.y();
	this->pose.orientation.z = quat.z();
	this->pose.orientation.w = quat.w();
}

bool ReachableCuboid::isAligned(Eigen::Vector3d& direction)
{
  double cos_theta = this->mean_direction.dot(direction) / (this->mean_direction.norm() * direction.norm());
	if(cos_theta > 1.0)
		cos_theta = 1.0;
	else if(cos_theta < -1.0)
		cos_theta = -1.0;

	double theta = std::acos(cos_theta);
	if(theta > angle_threshold)
		return false;
	else 
		return true;
}

bool ReachableCuboid::isAligned(Eigen::Vector3d& direction, double yaw)
{
	Eigen::Transform<double, 3, 0> T;
	T.translation() = Eigen::Vector3d::Zero();
	T.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
	Eigen::Vector3d transformed_direction = T * this->mean_direction;
  double cos_theta = transformed_direction.dot(direction) / (transformed_direction.norm() * direction.norm());
	if(cos_theta > 1.0)
		cos_theta = 1.0;
	else if(cos_theta < -1.0)
		cos_theta = -1.0;

	double theta = std::acos(cos_theta);
	if(theta > angle_threshold)
		return false;
	else 
		return true;
}

bool ReachableCuboid::isInside(Eigen::Vector3d& pos)
{
	T_B2Cub.setOrigin(tf::Vector3(this->center.x(), this->center.y(), this->center.z()));
	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(this->rotations.z(), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(this->rotations.y(), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(this->rotations.x(), Eigen::Vector3d::UnitX());
	Eigen::Quaterniond quat(R);
	T_B2Cub.setRotation(tf::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
	
	tf::Vector3 pos_tf_B = eigen2Tf(pos);
	tf::Vector3 pos_tf_Cub = T_B2Cub.inverse() * pos_tf_B;
	
	Eigen::Vector3d pos_Cub = tf2Eigen(pos_tf_Cub);
	if((this->size(0)/2.0 - std::abs(pos_Cub.x())) >= 0.0 &&
			(this->size(1)/2.0 - std::abs(pos_Cub.y())) >= 0.0 &&
			(this->size(2)/2.0 - std::abs(pos_Cub.z())) >= 0.0)
	{
		return true;
	}
	else
		return false;
}

visualization_msgs::Marker ReachableCuboid::getMarker(const Eigen::Vector3d& base_pos, const double& yaw, int color, std::string frame_id)
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();
	marker.ns = "reachable_cuboid_" + std::to_string(color);
	marker.id = color;
	marker.type = visualization_msgs::Marker::CUBE;
	marker.action = visualization_msgs::Marker::ADD;

	tf::Transform T_W2B;
	T_W2B.setOrigin(tf::Vector3(base_pos.x(), base_pos.y(), base_pos.z()));
	Eigen::Matrix3d R_W2B;
	R_W2B = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).matrix();
	Eigen::Quaterniond quat_W2B(R_W2B);
	T_W2B.setRotation(tf::Quaternion(quat_W2B.x(), quat_W2B.y(), quat_W2B.z(), quat_W2B.w()));
	tf::Vector3 cub_pos_W = T_W2B * eigen2Tf(this->center);
	
	marker.pose.position.x = cub_pos_W.getX();
	marker.pose.position.y = cub_pos_W.getY();
	marker.pose.position.z = cub_pos_W.getZ();

	// std::cout << "cub_pos_W: " << cub_pos_W.getX() << " " << cub_pos_W.getY() << " " << cub_pos_W.getZ() << std::endl;

	Eigen::Matrix3d R, R1, R2;
	R1 = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
	R2 = Eigen::AngleAxisd(this->rotations.z(), Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(this->rotations.y(), Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(this->rotations.x(), Eigen::Vector3d::UnitX());
	R = R2 * R1;
	Eigen::Quaterniond quat(R);
	marker.pose.orientation.x = quat.x();
	marker.pose.orientation.y = quat.y();
	marker.pose.orientation.z = quat.z();
	marker.pose.orientation.w = quat.w();
	marker.scale.x = this->size(0);
	marker.scale.y = this->size(1);
	marker.scale.z = this->size(2);
	marker.color.a = 0.4;
	marker.color.r = (color == 0) ? 1.0 : 0.0;
	marker.color.g = (color == 1) ? 1.0 : 0.0;
	marker.color.b = (color == 2) ? 1.0 : 0.0;
	
	return marker;
}

Eigen::Vector3d tf2Eigen(const tf::Vector3 &v_tf)
{
	return Eigen::Vector3d(v_tf.x(), v_tf.y(), v_tf.z());
}

tf::Vector3 eigen2Tf(const Eigen::Vector3d &v)
{
	return tf::Vector3(v.x(), v.y(), v.z());
}

geometry_msgs::Pose eigen2Pose(const Eigen::Vector3d &center, const Eigen::Vector3d &direction)
{
	geometry_msgs::Pose pose;

	pose.position.x = center.x();
	pose.position.y = center.y();
	pose.position.z = center.z();

	double r = direction.head(2).norm();
	double phi = atan2(direction.z(), r);
	double theta = atan2(direction.y(), direction.x());

	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) *
			Eigen::AngleAxisd(-phi, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
	
	Eigen::Quaterniond quat(R);

	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();
	pose.orientation.w = quat.w();

	return pose;
}

geometry_msgs::Pose eigen2Pose(const Eigen::Vector3d &center, const double yaw)
{
	geometry_msgs::Pose pose;

	pose.position.x = center.x();
	pose.position.y = center.y();
	pose.position.z = center.z();

	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond quat(R);
	pose.orientation.w = quat.w();
	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();

	return pose;
}