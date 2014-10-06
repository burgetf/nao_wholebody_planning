/*
 * planning_service_center.cpp
 *
 *  Created on: Aug 14, 2012
 *      Author: Felix Burget
 */


#include <rrt_connect_planner/planning_service_center.h>

namespace planner_control {


	Planning_Service_Center::Planning_Service_Center()
	{
		ros::NodeHandle nh;

		//Bind Services
		generate_ds_database_ = nh.advertiseService("generate_ds_database", &NAO_PLANNER_CONTROL::generate_ds_database, &nao_planner_control);
		get_goal_config_ = nh.advertiseService("compute_goal_config", &NAO_PLANNER_CONTROL::compute_goal_config, &nao_planner_control);
		motion_planning_ = nh.advertiseService("compute_motion_plan", &NAO_PLANNER_CONTROL::compute_motion_plan, &nao_planner_control);
		linear_manipulation_planning_ = nh.advertiseService("compute_linear_manipulation_plan", &NAO_PLANNER_CONTROL::compute_linear_manipulation_plan, &nao_planner_control);
		circular_manipulation_planning_ =  nh.advertiseService("compute_circular_manipulation_plan", &NAO_PLANNER_CONTROL::compute_circular_manipulation_plan, &nao_planner_control);
	}


	Planning_Service_Center::~Planning_Service_Center()
	{

	}


}
