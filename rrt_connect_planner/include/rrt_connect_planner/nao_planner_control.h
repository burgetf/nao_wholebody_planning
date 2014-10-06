 /*
 * nao_planner_control.h
 *
 *  Created on: Aug 14, 2012
 *      Author: Felix Burget
 */

#ifndef NAO_PLANNER_CONTROL_H_
#define NAO_PLANNER_CONTROL_H_


#include <ros/ros.h>
#include <rrt_connect_planner/rrt_planner.h>
#include <rrt_connect_planner/stable_config_generator.h>

#include <rrt_planner_msgs/Generate_DS_Configs.h>
#include <rrt_planner_msgs/Compute_Goal_Config.h>
#include <rrt_planner_msgs/Compute_Motion_Plan.h>
#include <rrt_planner_msgs/Compute_Linear_Manipulation_Plan.h>
#include <rrt_planner_msgs/Compute_Circular_Manipulation_Plan.h>





namespace planner_control {


class NAO_PLANNER_CONTROL
{
	public:
	NAO_PLANNER_CONTROL();
	~NAO_PLANNER_CONTROL();


	//Service Callbacks
	bool generate_ds_database(rrt_planner_msgs::Generate_DS_Configs::Request &req, rrt_planner_msgs::Generate_DS_Configs::Response &resp);
	bool compute_goal_config(rrt_planner_msgs::Compute_Goal_Config::Request &req, rrt_planner_msgs::Compute_Goal_Config::Response &resp);
	bool compute_motion_plan(rrt_planner_msgs::Compute_Motion_Plan::Request &req, rrt_planner_msgs::Compute_Motion_Plan::Response &resp);
	bool compute_linear_manipulation_plan(rrt_planner_msgs::Compute_Linear_Manipulation_Plan::Request &req, rrt_planner_msgs::Compute_Linear_Manipulation_Plan::Response &resp);
	bool compute_circular_manipulation_plan(rrt_planner_msgs::Compute_Circular_Manipulation_Plan::Request &req, rrt_planner_msgs::Compute_Circular_Manipulation_Plan::Response &resp);



	protected:
	//Node Handle
	ros::NodeHandle nh_;

	//Planning Scene Monitor
	boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;

    //Planning Scene Monitor
    boost::shared_ptr<planning_scene::PlanningScene> p_s_;

	//Stable Configuration Generator
	boost::shared_ptr<nao_constraint_sampler::StableConfigGenerator> scg_;

	//RRT Planner
	boost::shared_ptr<nao_planner::RRT_Planner> planner_;


	//GENERAL PARAMS
	std::string ROBOT_DESCRIPTION;      // name of the robot description (a param name, so it can be changed externally)
	std::string PLANNING_GROUP;  		// group to plan for
	double SCALE_SP ;					//Scaling factor for the support polygon (scaling to avoid projected COM to reach SP border)


	//STABLE CONFIG GENERATOR PARAMS
	int MAX_SAMPLES;
	int MAX_IK_ITERATIONS;


	//RRT CONNECT PLANNER PARAMS
	double ADVANCE_STEPS;
	int MAX_EXPAND_ITERATIONS;

	//JOINT WEIGHTS FOR MOTION PLANS
	double R_ARM_WEIGHT_APPROACH;
	double L_ARM_WEIGHT_APPROACH;
	double LEGS_WEIGHT_APPROACH;
	double R_ARM_WEIGHT_MANIPULATE;
	double L_ARM_WEIGHT_MANIPULATE;
	double LEGS_WEIGHT_MANIPULATE;

	//FIXED PARAMS (not from Server)
    char* SSC_DS_CONFIGS; 			//File DS database
    char* SSC_DS_GOAL_CONFIG_FIRST; 	//File Goal Configs for first motion plan
    char* SSC_DS_GOAL_CONFIG_SECOND; 	//File Goal Configs for second motion plan
    char* SOLUTION_FILE_PATH_FIRST;	//File containing configs of solution path
    char* SOLUTION_FILE_PATH_SECOND;  //File containing configs of solution path




    //Joint Names Array and function to bring the order of the joints in the RobotModel in the order used for planning
    std::vector<std::string> joint_names_;
    std::vector<std::string> joint_name_order_RobModel_to_Planner(const std::vector<std::string> j_names);


};

}/* namespace planner_control */
#endif /* NAO_PLANNER_CONTROL_H_ */
