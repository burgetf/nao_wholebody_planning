/*
 * stable_config_generator.h
 *
 *  Created on: May 10, 2012
 *      Author: Felix Burget
 */

#ifndef STABLE_CONFIG_GENERATOR_H_
#define STABLE_CONFIG_GENERATOR_H_

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <hrl_kinematics/TestStability.h>

#include "rrt_connect_planner/nao_constraint_kinematics.h"
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

#include <fstream>

#include <rrt_planner_msgs/Generate_DS_Configs.h>


namespace nao_constraint_sampler {

class StableConfigGenerator
{
public:
    //StableConfigGenerator(boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm, const std::string &group_name, float scale_sp) ;
    StableConfigGenerator(boost::shared_ptr<planning_scene::PlanningScene> p_s, const std::string &group_name, float scale_sp) ;
	virtual ~StableConfigGenerator();

	//Check further constraints -> considered in the isStateValid function
    //bool isFeasible(const robot_state::RobotState& state, bool verbose);
    bool isFeasible(robot_state::RobotState state, bool verbose);

	//Specify the desired position of the right hand (the orientation is such that the y axis of the hand and the r_foot are parallel)
	void setDesiredRightArmPose(std::vector<double> right_hand_pose);

	//Sample statically stable configurations / or stat.- stable goal configurations
    int sample_stable_configs(int max_samples, int max_ik_iterations , float &time_elapsed, bool sample_goal_configs, char* file_destination);


	//Load the best goal config found
    void loadGoalConfig(std::vector<double>& config, char* file_destination);



protected:
	ros::NodeHandle root_nh_;
	ros::NodeHandle nh_;

    //Planning Scene Monitor
    //boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;


    //Planning Scene Monitor
    boost::shared_ptr<planning_scene::PlanningScene> p_s_;

	//Constraint sampler
	//constraint_samplers::ConstraintSampler cs_;

	//Stability constraint
	hrl_kinematics::TestStability stability_;

	//Name of Planning Group
	std::string group_name_;

	//From constraint_sampler.h
    //planning_scene::PlanningSceneConstPtr                   scene_;

	//Random number generator for generating random configs
	random_numbers::RandomNumberGenerator                   random_number_generator_;

	//Kinematics FK + IK for legs (guarantees double support)
	ConstraintKinematics leg_kinematics_;

	//Files to store configurations
	std::ofstream stat_stable_samples_DS_;
	std::ofstream stat_stable_goal_samples_DS_;

	//Sort goal configurations by the ergonomy index
    void sort_goal_configs_by_ergonomy(char* file_destination);


    //Joint Names Array and function to bring the order of the joints in the RobotModel in the order used for planning
    std::vector<std::string> joint_names_;
    std::vector<std::string> joint_name_order_RobModel_to_Planner(const std::vector<std::string> j_names);



};

} /* namespace nao_constraint_sampler */
#endif /* STABLE_CONFIG_GENERATOR_H_ */
