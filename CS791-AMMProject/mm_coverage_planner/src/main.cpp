// #include "utils/data_loader.hpp"
//#include "viewpoint_grouper.hpp"
#include "path_planner.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <random>
#include <time.h>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "test_node");
    ros::AsyncSpinner spinner(3); // TODO: Examples have 1, but move_group fails with: "Didn't received robot state (joint angles) with recent timestamp within 1 seconds." (see: https://github.com/ros-planning/moveit/issues/868)
    spinner.start();

  	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");
	std::srand(std::time(0));
	std::shared_ptr<SharedData> shared_data = std::make_shared<SharedData>();
	//std::vector<int> route;
	DataLoader data_loader(shared_data, nh);
	// data_loader.loadViewPoints("/home/mihir/autonomous_mobile_manipulation_ws/src/CS791-AMMProject/mm_coverage_planner/config/viewpoints");
	data_loader.loadFullData(ros::package::getPath("mm_coverage_planner") + "/config/full_data");
	data_loader.visualizeViewpoints();
	data_loader.testSubgroups();

	ViewpointGrouper grouper(shared_data, nh);
	grouper.loadDummyReachableSpace();
	grouper.formGroups();
	
	PathPlanner pplanner(shared_data, nh, private_nh, "map");
	pplanner.baseOptPath();
	pplanner.visualizeBaseLocations();
	
    ros::waitForShutdown();
    return 0;

}
