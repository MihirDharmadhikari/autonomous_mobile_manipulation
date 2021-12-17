#include "path_planner.hpp"

PathPlanner::PathPlanner(std::shared_ptr<SharedData> shared_data, const ros::NodeHandle& nh, const ros::NodeHandle& private_nh, std::string world_link="map")
		:nh_(nh), private_nh_(private_nh), world_link_(world_link), shared_data_(shared_data) 
{
	base_loc_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/bvr_SIM/tg_base_locations", 10, true);
	bvr_goal_pub_ = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/bvr_SIM/move_base/goal", 10, true);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/bvr_SIM/bvr_SIM_velocity_controller/cmd_vel", 10, true);
  bvr_goal_stat_sub_ = nh_.subscribe("/bvr_SIM/move_base/result", 10, &PathPlanner::goalCallBack, this);
  //arm_joint_sub_ = nh_.subscribe("", 10, &PathPlanner::goalCallBack, this);
  armInitialize();
}

/**
 * @brief PathPlanner::visualizeBaseLocations()
 * 
 * 
 */
void PathPlanner::visualizeBaseLocations()
{
	base_location_counter_=0;
  vp_counter_ = 0;
	if(base_location_counter_ == 0)
	{	
		ROS_INFO("base_location_counter_ = 0 case CHECK\n");
		move_base_msgs::MoveBaseActionGoal tg_all_base_locations;
		tg_all_base_locations.goal.target_pose.header.frame_id = "map";//shared_data_->world_frame_;
		tg_all_base_locations.goal.target_pose.pose.position.x = shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.position.x;
		tg_all_base_locations.goal.target_pose.pose.position.y = shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.position.y;
		tg_all_base_locations.goal.target_pose.pose.position.z = shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.position.z;
		tg_all_base_locations.goal.target_pose.pose.orientation.x = shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.orientation.x;
		tg_all_base_locations.goal.target_pose.pose.orientation.y = shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.orientation.y;
		tg_all_base_locations.goal.target_pose.pose.orientation.z = shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.orientation.z;
		tg_all_base_locations.goal.target_pose.pose.orientation.w = shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.orientation.w; 
    //std::cout<<"base_route_[0]: "<<base_route_[0]<<"\n"<<std::endl;
    //std::cout<<"shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.position.x "<<shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.position.x<<" "<<std::endl;
    //std::cout<<"shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.position.y "<<shared_data_->viewpoint_groups_[base_route_[0]-1].base_pose.position.y<<" "<<std::endl;
		bvr_goal_pub_.publish(tg_all_base_locations);
	}
	ROS_INFO("ROS_CHECK\n");
	ROS_INFO("Total number of base locations: %i", shared_data_->viewpoint_groups_.size());
}

/**
 * @brief Algorithm
 * This callBack is called when msg.status.status is 3 (SUCCEEDED)
 * which means the mover reaches to the goal. 
 *  
  * Once the goal is reached (status=3), goalCallBack() is triggered
  * Read cuboid viewpoints which are associated to the current base location
  * moveit plans motion route of the end-effector
  * Manipulator visits viewpoints locations within the cuboid (this can be done by for-loop)
  * Once all the viewpoints are visited, program continues and mover moves to the next base location.
  * This continues untill all the locations visited.
  * 
  */
void PathPlanner::goalCallBack(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
	ROS_INFO("goalCallBack is active\n");
	goalStatus=0;
	goalStatus = msg->status.status;
	ROS_INFO("status_list[0]: %i", goalStatus);//.toString().c_str());
	ROS_INFO("base_location_counter_= %i", base_location_counter_);
	ROS_INFO("\n");

  if(goalStatus != 3)
  {
    geometry_msgs::Pose tgt_pose = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose;
    for(int i=0;i<100;++i)
    {
      geometry_msgs::Twist vel;
      vel.linear.x = -0.4;
      cmd_vel_pub_.publish(vel);
      ros::Duration(0.01).sleep();
    }

    move_base_msgs::MoveBaseActionGoal tg_all_base_locations;
    tg_all_base_locations.goal.target_pose.header.frame_id = "map";//shared_data_->world_frame_;
    tg_all_base_locations.goal.target_pose.pose.position.x = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.position.x;
    tg_all_base_locations.goal.target_pose.pose.position.y = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.position.y;
    tg_all_base_locations.goal.target_pose.pose.position.z = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.position.z;
    tg_all_base_locations.goal.target_pose.pose.orientation.x = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.x;
    tg_all_base_locations.goal.target_pose.pose.orientation.y = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.y;
    tg_all_base_locations.goal.target_pose.pose.orientation.z = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.z;
    tg_all_base_locations.goal.target_pose.pose.orientation.w = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.w; //+1.5708;
    bvr_goal_pub_.publish(tg_all_base_locations);

    return;
  }

	// Read viewpoints within the cuboid associated with the base location
	move_base_msgs::MoveBaseActionGoal target_view_point;
	target_view_point.goal.target_pose.header.frame_id = "map";
  geometry_msgs::PoseStamped viewpoint_loc;
  viewpoint_loc.header.frame_id = "map";
  geometry_msgs::PointStamped viewpoint_transformed;
	if (move_group_) 
	{
    move_group_->stop();
  }
	else {
		ROS_WARN_STREAM("move_group_interface: move_group_ not constructed yet"); 
		throw std::runtime_error("move_group_interface: move_group_ not constructed yet");
	}
	// if(((base_location_counter_==0 && goalStatus != 3)|| (base_location_counter_ != 0 && goalStatus == 3)) && base_location_counter_ < shared_data_->viewpoint_groups_.size())
  if(base_location_counter_ > shared_data_->viewpoint_groups_.size())
  {
    ROS_INFO("Mission completed");
    // Base returns to starting point
    move_base_msgs::MoveBaseActionGoal tg_all_base_locations;
    tg_all_base_locations.goal.target_pose.header.frame_id = "map";
    tg_all_base_locations.goal.target_pose.pose.position.x = 0.0;
    tg_all_base_locations.goal.target_pose.pose.position.y = 0.0;
    tg_all_base_locations.goal.target_pose.pose.position.z = 0.218416;
    tg_all_base_locations.goal.target_pose.pose.orientation.x = 0.0;
    tg_all_base_locations.goal.target_pose.pose.orientation.y = 0.0;
    tg_all_base_locations.goal.target_pose.pose.orientation.z = 0.0;
    tg_all_base_locations.goal.target_pose.pose.orientation.w = 1.0;
    bvr_goal_pub_.publish(tg_all_base_locations);    
    return;
  }
  for(int i=0; i<shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups.size(); ++i)
  {
    for(int j=0; j<shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i].size(); ++j)
    {
      ROS_INFO("base_location_counter_: %i", base_location_counter_);
      ROS_INFO("PP: Inside the if statement");
      viewpoint_loc.pose.position.x = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i][j].pose.position.x;
      viewpoint_loc.pose.position.y = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i][j].pose.position.y;
      viewpoint_loc.pose.position.z = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i][j].pose.position.z;
      viewpoint_loc.pose.orientation.x = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i][j].pose.orientation.x;
      viewpoint_loc.pose.orientation.y = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i][j].pose.orientation.y;
      viewpoint_loc.pose.orientation.z = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i][j].pose.orientation.z;
      viewpoint_loc.pose.orientation.w = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].viewpoint_subgroups[i][j].pose.orientation.w;
      viewpoint_loc.header.stamp = ros::Time::now();
      base_loc_pub_.publish(viewpoint_loc);
      robot_state::RobotState start_state(*move_group_->getCurrentState());
      const robot_state::LinkModel* ee_link_model = start_state.getLinkModel(ee_link_);
      // We will reuse the old goal that we had and plan to it. 
      // Note that this will only work if the current state already satisfies 
      // the path constraints. So, we need to set the start state to a new pose.
      geometry_msgs::Pose start_pose = move_group_->getCurrentPose().pose;
      start_state.setFromIK(joint_model_group_, start_pose);
      move_group_->setStartState(start_state);
      // Now we will plan to the earlier pose target from the new start state 
      // that we have just created.
      move_group_->setPoseTarget(viewpoint_loc, ee_link_);
      // Planning with constraints can be slow because every sample must call 
      // an inverse kinematics solver.
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan (constraints) %s", success ? "SUCCESSFULL" : "FAILED");
      if(!success)
      {
        ++unreachable_vps_counter_;
        std::cout << "Unreachable vps: " << unreachable_vps_counter_ << "/" << total_vps_counter_ << std::endl;
        continue;
      }

      // Visualize the plan in RViz
      geometry_msgs::PoseStamped target_pose_VT;
      try
      {
        tf_listener_->transformPose(visual_tools_->getBaseFrame(), viewpoint_loc, target_pose_VT);
        visual_tools_->publishAxisLabeled(target_pose_VT.pose, "ee_ref_pose");
        visual_tools_->publishTrajectoryLine(my_plan.trajectory_, joint_model_group_);
        visual_tools_->publishTrajectoryPath(my_plan.trajectory_, move_group_->getCurrentState());
        visual_tools_->trigger();
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
        ROS_WARN("Can't visualize plan");
      }
      bool exec_success = (move_group_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Execution of plan %s", exec_success ? "SUCCESSFULL" : "FAILED");
      ROS_INFO("Goal published for base_location_counter_= %i", base_location_counter_);
    }
	}
  
  std::cout << "Visited viewpoints for this base. Moving on" << std::endl;

  for(int i=0;i<100;++i)
  {
    geometry_msgs::Twist vel;
    vel.linear.x = -0.5;
    cmd_vel_pub_.publish(vel);
    ros::Duration(0.02).sleep();
  }

  base_location_counter_++;
  move_base_msgs::MoveBaseActionGoal tg_all_base_locations;
  tg_all_base_locations.goal.target_pose.header.frame_id = "map";
  tg_all_base_locations.goal.target_pose.pose.position.x = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.position.x;
  tg_all_base_locations.goal.target_pose.pose.position.y = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.position.y;
  tg_all_base_locations.goal.target_pose.pose.position.z = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.position.z;
  tg_all_base_locations.goal.target_pose.pose.orientation.x = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.x;
  tg_all_base_locations.goal.target_pose.pose.orientation.y = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.y;
  tg_all_base_locations.goal.target_pose.pose.orientation.z = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.z;
  tg_all_base_locations.goal.target_pose.pose.orientation.w = shared_data_->viewpoint_groups_[base_route_[base_location_counter_]-1].base_pose.orientation.w; //+1.5708;
  bvr_goal_pub_.publish(tg_all_base_locations);
	ROS_INFO("goalCallBak end \n");
}

/**
 * @brief armInitialize() initializes arm moveit package
 * 
 */

void PathPlanner::armInitialize()
{
  std::string robot_namespace;
  private_nh_.getParam("robot_namespace", robot_namespace);
  std::string arm_namespace;
  private_nh_.getParam("arm_namespace", arm_namespace);
  // Here, we create a TransformListener object. Once the listener is created, it starts receiving
  // tf transformations over the wire, and buffers them up to 10 seconds. The TransformListener
  // object should be scoped to persist otherwise it's cache will be unable to fill and almost every
  // query will fail. A common method is to make the TransformListener ojbect a member variable of a class.
  tf_listener_ = std::make_unique<tf::TransformListener>();
  // Setup Visualization
  visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(robot_namespace + "/bvr_base_link");
  visual_tools_->deleteAllMarkers();
  // visual_tools->loadRemoteControl();
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools_->trigger();
  // Setup Interface
  PLANNING_GROUP_ = arm_namespace;  //Assumes arm_namespace:=moveit planning group 
  std::cout<<"PLANNING_GROUP_: "<<robot_namespace<<std::endl;
  move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP_);
  planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
  joint_model_group_ = move_group_->getCurrentState()->getJointModelGroup(PLANNING_GROUP_);
  ROS_INFO_NAMED(move_group::NODE_NAME, "Robot Model name: [%s] at frame: [%s]", move_group_->getRobotModel()->getName().c_str(), move_group_->getRobotModel()->getModelFrame().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "ROBOT MODEL-based / SRDF-based Pose Reference frame: [%s]", move_group_->getPoseReferenceFrame().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "ROBOT MODEL-based / SRDF-based Planning frame: [%s]", move_group_->getPlanningFrame().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "SRDF-based End-Effector link: [%s]", move_group_->getEndEffectorLink().c_str());
  ROS_INFO_NAMED(move_group::NODE_NAME, "Available Planning Groups:");
  const std::vector<std::string>& joint_model_group_names = move_group_->getJointModelGroupNames();
  std::copy(joint_model_group_names.begin(), joint_model_group_names.end(), std::ostream_iterator<std::string>(std::cout, " "));
  plan_link_ = move_group_->getPlanningFrame();
  std::cout << "plan_link_: " << plan_link_ << std::endl;
  ee_link_ = robot_namespace + "/" + arm_namespace + "/gripper_manipulation_link";
  // Necessary if planning is to take place in frame other than move_group_->getPlanningFrame() - a.k.a. base_link
  // move_group_->setPoseReferenceFrame(robot_namespace + "/bvr_base_link");
  // visual_tools_->setLifetime(30.0);
  // Reduce the speed of the robot arm via a scaling factor of the maxiumum speed of each joint. Note this is not the speed of the end effector point.
  move_group_->setMaxVelocityScalingFactor(0.25);
  // Set a scaling factor for optionally reducing the maximum joint acceleration.
  move_group_->setMaxAccelerationScalingFactor(1.0);
  // Planning with constraints can be slow because every sample must call an inverse kinematics solver (default 5 seconds)
  move_group_->setPlanningTime(5.0); //5.0
  // Number of times the motion plan is to be computed from scratch before the shortest solution is returned. 
  move_group_->setNumPlanningAttempts(100); //10	
  // Set the tolerance that is used for reaching the goal. For joint state goals, this will be distance for each joint, in the configuration space (radians or meters depending on joint type). For pose goals this will be the radius of a sphere where the end-effector must reach.
  move_group_->setGoalTolerance(0.02);
  // Pick one of the available configs - see ee ompl_planning<_SIM>.yaml for a complete list
  move_group_->setPlannerId("RRTConnectkConfigDefault");
  ROS_INFO_STREAM_NAMED(move_group::NODE_NAME, "Available Planning Params for " << move_group_->getPlannerId() << ":");
  std::map<std::string,std::string> planner_params = move_group_->getPlannerParams(move_group_->getPlannerId(), PLANNING_GROUP_);
  for (const auto & param : planner_params) {  std::cout << " - " << param.first << " : " << param.second << std::endl; }
  ROS_INFO_STREAM_NAMED(move_group::NODE_NAME, "Setup complete!");
}

/**
 * @brief baseOptPath() is to find the optimal path around the boat by utilizing Traveller Salesman Problem approach
 * tspCost[][] matrix contains the distance between base locations. 
 * Greedy approach allows the robot move to the nearest base location. Hence, the overall goal is to visit 
 * all the base locations in an order around the boat.
 * 
 */
void PathPlanner::baseOptPath()
{
  std::cout<<"/******Traveller Salesman Problem -> Optimal Base Path Finder******/"<<"\n"<<std::endl;
  int groupingSize = shared_data_->viewpoint_groups_.size();
  std::vector<std::vector<float> > tspCost(groupingSize+1, std::vector<float>(groupingSize+1, 0.0f));
  //std::cout<<"groupingSize: "<<shared_data_->viewpoint_groups_.size()<<"\n"<<std::endl;
  for (int i=0; i<=shared_data_->viewpoint_groups_.size(); i++)
  {
    for (int j=0; j<=shared_data_->viewpoint_groups_.size(); j++)
    {
      if(i == 0 && j >0)
      {
        tspCost[i][j] = (float)std::sqrt((float)std::pow(0.0f - (float)shared_data_->viewpoint_groups_[j-1].base_pose.position.x, 2) + std::pow(0.0f - (float)shared_data_->viewpoint_groups_[j-1].base_pose.position.y, 2));
        //std::cout<<"tspCost["<<i<<"]["<<j<<"]: "<<tspCost[i][j]<<" "<<std::endl;
      }
      else if (i > 0 && j ==0)
      {
        tspCost[i][j] = (float)std::sqrt((float)std::pow((float)shared_data_->viewpoint_groups_[i-1].base_pose.position.x - 0.0f, 2) + std::pow((float)shared_data_->viewpoint_groups_[i-1].base_pose.position.y - 0.0f, 2));
        //std::cout<<"tspCost["<<i<<"]["<<j<<"]: "<<tspCost[i][j]<<" "<<std::endl;
      }
      else
      {
        tspCost[i][j] = (float)std::sqrt((float)std::pow((float)shared_data_->viewpoint_groups_[i-1].base_pose.position.x - (float)shared_data_->viewpoint_groups_[j-1].base_pose.position.x, 2) + std::pow((float)shared_data_->viewpoint_groups_[i-1].base_pose.position.y - (float)shared_data_->viewpoint_groups_[j-1].base_pose.position.y, 2));
        //std::cout<<"tspCost["<<i<<"]["<<j<<"]: "<<tspCost[i][j]<<" "<<std::endl;
        //std::cout<<"shared_data_->viewpoint_groups_[i-1].base_pose.position.x "<<shared_data_->viewpoint_groups_[i-1].base_pose.position.x<<" "<<std::endl;
        //std::cout<<"shared_data_->viewpoint_groups_[j-1].base_pose.position.x "<<shared_data_->viewpoint_groups_[j-1].base_pose.position.x<<" "<<std::endl;
        //std::cout<<"shared_data_->viewpoint_groups_[i-1].base_pose.position.y "<<shared_data_->viewpoint_groups_[i-1].base_pose.position.y<<" "<<std::endl;
        //std::cout<<"shared_data_->viewpoint_groups_[j-1].base_pose.position.y "<<shared_data_->viewpoint_groups_[j-1].base_pose.position.y<<" "<<std::endl;
      }      
    }
  }
  // Finding base optimal path by TSP
  base_tsp_flag = 1;
  tsp(tspCost);
  base_tsp_flag = 0;
}

/**
 * @brief armOptPath() is to find the optimal path for visiting viewpoints within the cuboid
 * 
 * 
 */
void PathPlanner::armOptPath()
{
  std::cout<<"/******Traveller Salesman Problem -> Optimal Arm Path Finder******/"<<"\n"<<std::endl;
  //int vpSize = shared_data_->viewpoint_groups_[0].viewpoint_subgroups.size();
  //std::vector<std::vector<float> > tspCost(groupingSize+1, std::vector<float>(groupingSize+1, 0.0f));
  //std::cout<<"groupingSize: "<<shared_data_->viewpoint_groups_.size()<<"\n"<<std::endl;
}

/**
 * @brief tsp()
 * Generic Traveller Salesman Problem algorithm implementation
 * Greedy approach
 * Time complexity: O(N^2*logN)
 */
void PathPlanner::tsp(std::vector<std::vector<float>> tspCost) 
{
  float sum = 0.0f;
  int counter = 0;
  int row = 0;
  int col = 0;
  float min = 99.99f;
  std::map<int, int> visitedRouteList;
  visitedRouteList[0] = 1;
  int route[tspCost.size()];
  while (row < tspCost.size() && col < tspCost[row].size())
  {
    if (counter >= tspCost[row].size() - 1)
    {
        break;
    }
    if (col != row && (visitedRouteList[col] == 0))
    {
        if (tspCost[row][col] < min)
        {
            min = tspCost[row][col];
            route[counter] = col + 1;
        }
    }
    col++;
    if (col == tspCost[row].size())
    {
      sum += min;
      min = 99.99f;
      visitedRouteList[route[counter] - 1] = 1;
      col = 0;
      row = route[counter] - 1;
      counter++;   
    }
  }
  row = route[counter - 1] - 1;
  for (col = 0; col < tspCost.size(); col++)
  {
    if ((row != col) && tspCost[row][col] < min)
    {
        min = tspCost[row][col];
        route[counter] = col + 1;
    }
  }
  sum += min;
  //std::cout<<"Min Cost: "<<sum<<"\n"<<std::endl;
  base_route_[0]=0;
  if (base_tsp_flag == 1)
  {
    for (std::vector<std::vector<int>>::size_type i=0; i<tspCost.size(); i++)
    {
      base_route_[i] = route[i]-1;
      //std::cout<<"Optimal Path: "<<base_route_[i]<<' '<<"\n";
    }
  }
  else if (arm_tsp_flag==1)
  {
    for (std::vector<std::vector<int>>::size_type i=0; i<tspCost.size(); i++)
    {
      arm_route_[i] = route[i]-1;
      std::cout<<"Optimal Arm Path: "<<arm_route_[i]<<' '<<"\n";
    }
  }
  
}

/**
 * end of code
 * 
 */
