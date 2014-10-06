/*
 * nao_planner_control.cpp
 *
 *  Created on: Aug 14, 2012
 *      Author: Felix Burget
 */
 

#include <rrt_connect_planner/nao_planner_control.h>
#include <math.h>

namespace planner_control {


//Constructor
NAO_PLANNER_CONTROL::NAO_PLANNER_CONTROL(): nh_("~")
{

	//Read params from parameter server
	nh_.param("robot_description", ROBOT_DESCRIPTION, std::string("robot_description"));
	nh_.param("planning_group", PLANNING_GROUP, std::string("whole_body_rfoot"));
	nh_.param("scale_sp", SCALE_SP, 0.8);

	//Stable Configuration Generator Params
	nh_.param("scg_max_samples", MAX_SAMPLES, 3000);
	nh_.param("scg_max_ik_iterations", MAX_IK_ITERATIONS, 5);

	//RRT CONNECT Params
	nh_.param("planner_step_factor", ADVANCE_STEPS, 0.1);
	nh_.param("planner_max_iterations", MAX_EXPAND_ITERATIONS, 8000);
    //Joint weights
	nh_.param("r_arm_weight_approach", R_ARM_WEIGHT_APPROACH, 1.0);
	nh_.param("l_arm_weight_approach", L_ARM_WEIGHT_APPROACH, 1.0);
	nh_.param("legs_weight_approach", LEGS_WEIGHT_APPROACH, 1.0);
	nh_.param("r_arm_weight_manipulate", R_ARM_WEIGHT_MANIPULATE, 1.0);
	nh_.param("l_arm_weight_manipulate", L_ARM_WEIGHT_MANIPULATE, 1.0);
	nh_.param("legs_weight_manipulate", LEGS_WEIGHT_MANIPULATE, 1.0);


    //Path to files to read from and write into
    std::string ds_database;
    std::string goal_config_first;
    std::string goal_config_second;
    std::string solution_first;
    std::string solution_second;

    nh_.param("file_path_DS_database",ds_database,std::string("../config_database/ssc_double_support.dat"));
    nh_.param("file_path_first_goal_config",goal_config_first,std::string("../config_database/ssc_double_support_goal_first.dat"));
    nh_.param("file_path_second_goal_config",goal_config_second,std::string("../config_database/ssc_double_support_goal_second.dat"));
    nh_.param("file_path_solution_traj_first",solution_first,std::string("../solutions/solution_path_first.dat"));
    nh_.param("file_path_solution_traj_second",solution_second,std::string("../solutions/solution_path_second.dat"));


    SSC_DS_CONFIGS = new char[ds_database.size() + 1];
    std::copy(ds_database.begin(), ds_database.end(), SSC_DS_CONFIGS);
    SSC_DS_CONFIGS[ds_database.size()] = '\0'; // don't forget the terminating 0

    SSC_DS_GOAL_CONFIG_FIRST = new char[goal_config_first.size() + 1];
    std::copy(goal_config_first.begin(), goal_config_first.end(), SSC_DS_GOAL_CONFIG_FIRST);
    SSC_DS_GOAL_CONFIG_FIRST[goal_config_first.size()] = '\0'; // don't forget the terminating 0

    SSC_DS_GOAL_CONFIG_SECOND = new char[goal_config_second.size() + 1];
    std::copy(goal_config_second.begin(), goal_config_second.end(), SSC_DS_GOAL_CONFIG_SECOND);
    SSC_DS_GOAL_CONFIG_SECOND[goal_config_second.size()] = '\0'; // don't forget the terminating 0

    SOLUTION_FILE_PATH_FIRST = new char[solution_first.size() + 1];
    std::copy(solution_first.begin(), solution_first.end(), SOLUTION_FILE_PATH_FIRST);
    SOLUTION_FILE_PATH_FIRST[solution_first.size()] = '\0'; // don't forget the terminating 0


    SOLUTION_FILE_PATH_SECOND = new char[solution_second.size() + 1];
    std::copy(solution_second.begin(), solution_second.end(), SOLUTION_FILE_PATH_SECOND);
    SOLUTION_FILE_PATH_SECOND[solution_second.size()] = '\0'; // don't forget the terminating 0

//    //Set the file containing the statically stable configurations
//    SSC_DS_CONFIGS = ds_database.c_str();
//    //Set the file containing the statically stable GOAL configurations (for the first motion plan)
//    SSC_DS_GOAL_CONFIG_FIRST = goal_config_first.c_str();
//    //Set the file containing the statically stable GOAL configurations (for the second motion plan)
//    SSC_DS_GOAL_CONFIG_SECOND = goal_config_second.c_str();
//    //Files storing the solution paths
//    SOLUTION_FILE_PATH_FIRST = solution_first.c_str();
//    SOLUTION_FILE_PATH_SECOND = solution_second.c_str();


	//Create planning scene monitor
	psm_ = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

    //Create planning scene
    robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    p_s_ = boost::shared_ptr<planning_scene::PlanningScene>(new planning_scene::PlanningScene(kinematic_model));

	//Create Stable Config Generator
    //scg_ = boost::shared_ptr<nao_constraint_sampler::StableConfigGenerator>(new nao_constraint_sampler::StableConfigGenerator(psm_, PLANNING_GROUP, SCALE_SP));
    scg_ = boost::shared_ptr<nao_constraint_sampler::StableConfigGenerator>(new nao_constraint_sampler::StableConfigGenerator(p_s_, PLANNING_GROUP, SCALE_SP));

	//Create RRT Planner
    //planner_ = boost::shared_ptr<nao_planner::RRT_Planner>(new nao_planner::RRT_Planner(psm_,PLANNING_GROUP));
    planner_ = boost::shared_ptr<nao_planner::RRT_Planner>(new nao_planner::RRT_Planner(p_s_,PLANNING_GROUP));
	//Scale support polygon for first planner
	planner_->scale_support_polygon(SCALE_SP);
	//Load a set of statically stable configurations
	planner_->load_DS_database(SSC_DS_CONFIGS);

    //Fix Joint Names order
    //planning_scene_monitor::LockedPlanningSceneRO p_s(psm_);
    //const std::vector<std::string> joint_names = p_s->getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();
    //const std::vector<std::string> joint_names = psm_->getPlanningScene()->getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();
    const std::vector<std::string> joint_names = p_s_->getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();
    joint_names_ = joint_name_order_RobModel_to_Planner(joint_names);


	//Start Monitors of Planning_Scene_Monitor
	psm_->startWorldGeometryMonitor();
	psm_->startSceneMonitor();
	psm_->startStateMonitor();


}



//Destructor
NAO_PLANNER_CONTROL::~NAO_PLANNER_CONTROL() {

    delete[] SSC_DS_CONFIGS;
    delete[] SSC_DS_GOAL_CONFIG_FIRST;
    delete[] SSC_DS_GOAL_CONFIG_SECOND;
    delete[] SOLUTION_FILE_PATH_FIRST;
    delete[] SOLUTION_FILE_PATH_SECOND;
}




bool NAO_PLANNER_CONTROL::generate_ds_database(rrt_planner_msgs::Generate_DS_Configs::Request &req, rrt_planner_msgs::Generate_DS_Configs::Response &resp)
{
	//Sample statically stable configurations
	float time_for_first_config = 0.0;
	resp.num_configs_generated = scg_->sample_stable_configs(req.max_samples,req.max_ik_iterations,time_for_first_config, false, SSC_DS_CONFIGS);

	//Load the generated database into the planner
	planner_->load_DS_database(SSC_DS_CONFIGS);

	return true;
}



bool NAO_PLANNER_CONTROL::compute_goal_config(rrt_planner_msgs::Compute_Goal_Config::Request &req, rrt_planner_msgs::Compute_Goal_Config::Response &resp)
{
	std::vector<double> desired_right_hand_pose(6);
	//Set the desired position of the right hand w.r.t the right foot
	desired_right_hand_pose[0] = req.right_hand_position.x;   // X [m]
	desired_right_hand_pose[1] = req.right_hand_position.y; // Y [m]
	desired_right_hand_pose[2] = req.right_hand_position.z;  // Z [m]
	//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
	desired_right_hand_pose[3] = req.right_hand_direction.x;   // X component of hand z-axis
	desired_right_hand_pose[4] = req.right_hand_direction.y; // Y component of hand z-axis
	desired_right_hand_pose[5] = req.right_hand_direction.z;  // Z component of hand z-axis

	scg_->setDesiredRightArmPose(desired_right_hand_pose);
	float time_for_first_config = 0.0;
	scg_->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS, time_for_first_config,true, SSC_DS_GOAL_CONFIG_FIRST);

	//Get Joint Names of planning group
    //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();

	//Get the best goal config from the file
    std::vector<double> goal_config(joint_names_.size());
	scg_->loadGoalConfig(goal_config, SSC_DS_GOAL_CONFIG_FIRST);


	//Assign best goal config to service response
	resp.wb_goal_config = goal_config;

	return true;
}



bool NAO_PLANNER_CONTROL::compute_motion_plan(rrt_planner_msgs::Compute_Motion_Plan::Request &req, rrt_planner_msgs::Compute_Motion_Plan::Response &resp)
{

    std::cout <<"Compute Manipulation Plan requested"<< std::endl;

//	//-----------Start Configuration (= current robot state in planning_scene)
//	//Get the joint names of the planning group
//    //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();

//	//Get the current configuration (for the whole robot)
//    robot_state::RobotState state = p_s_->getCurrentState();


//    //Get Joint Names
//    std::vector<std::string> joint_title;
//    joint_title = state.getVariableNames();


//    //Get Joint Values
//    const double *joint_vals;
//    joint_vals = state.getVariablePositions();

//    std::map< std::string, double > joint_values;
//    for (int i = 0; i < joint_title.size(); i++)
//    {
//        joint_values.insert ( std::pair<std::string,double>(joint_title[i],joint_vals[i]));
//    }
//    //state.getStateValues(joint_values);


//	//Erase configuration elements not used for planning
//	joint_values.erase("HeadPitch");
//	joint_values.erase("HeadYaw");
//	joint_values.erase("world_joint/theta");
//	joint_values.erase("world_joint/x");
//	joint_values.erase("world_joint/y");

//    //    //Plot remaining configuration
//    //        for(std::map<std::string,double>::const_iterator i = joint_values.begin(); i != joint_values.end(); ++i)
//    //        {
//    //            std::cout << i->first << ": " << i->second << std::endl;
//    //        }



//	//Configuration for Planning (rearrange joint values in the order of the Kin. Model)
//    std::vector<double> start_config(joint_names_.size());
//    std::cout<<"Start Config."<<std::endl;
//    for(unsigned int i = 0 ; i < joint_names_.size(); i++)
//	{
//     start_config[i] = joint_values[joint_names_[i]];
//     std::cout << joint_names_[i] << ": " << joint_values[joint_names_[i]] << std::endl;
//	}

    //-----------Start Configuration (from request) -------------------------
    std::vector<double> start_config(joint_names_.size());
    std::cout<<"Start Config."<<std::endl;
    for(unsigned int i = 0 ; i < joint_names_.size(); i++)
    {
     start_config[i] = req.wb_start_config[i];
     std::cout << joint_names_[i] << ": " << start_config[i] << std::endl;
    }


	//Set trajectory mode (false will prepare a trajectory for simulation, true will prepare a trajectory for execution)
	planner_->setTrajectoryMode(req.execute_trajectory);

	//-----------Goal Configuration (generated by SCG)
	std::vector<double> desired_right_hand_pose(6);
	//Set the desired position of the right hand w.r.t the right foot
	desired_right_hand_pose[0] = req.right_hand_position.x;   // X [m]
	desired_right_hand_pose[1] = req.right_hand_position.y; // Y [m]
	desired_right_hand_pose[2] = req.right_hand_position.z;  // Z [m]
	//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
	desired_right_hand_pose[3] = req.right_hand_direction.x;   // X component of hand z-axis
	desired_right_hand_pose[4] = req.right_hand_direction.y; // Y component of hand z-axis
	desired_right_hand_pose[5] = req.right_hand_direction.z;  // Z component of hand z-axis

	//Sample a goal configuration
	scg_->setDesiredRightArmPose(desired_right_hand_pose);
	int num_goal_configs_found = 0;
	float time_for_first_config = 0.0;
	num_goal_configs_found = scg_->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, true, SSC_DS_GOAL_CONFIG_FIRST);
	//Time required to find first config
	resp.time_goal_config = time_for_first_config;

	if (num_goal_configs_found > 0)
	{
		//Get the best goal config from the file
        std::vector<double> goal_config(joint_names_.size());
		scg_->loadGoalConfig(goal_config, SSC_DS_GOAL_CONFIG_FIRST);

		//Assign best goal config to service response
		resp.wb_goal_config = goal_config;


		//-----------Planner
		//Set joint weights
        std::vector<double> joint_weights(joint_names_.size());
		//Set joint weights
		double r_arm_weight = R_ARM_WEIGHT_APPROACH;
		double l_arm_weight = L_ARM_WEIGHT_APPROACH;
		double leg_weight = LEGS_WEIGHT_APPROACH;
		joint_weights[0] = leg_weight;			//	RAnkleRoll
		joint_weights[1] = leg_weight;			//	RAnklePitch
		joint_weights[2] = leg_weight;			//	RKneePitch
		joint_weights[3] = leg_weight;			//	RHipPitch
		joint_weights[4] = leg_weight;			//	RHipRoll
		joint_weights[5] = l_arm_weight;		//	LShoulderPitch
		joint_weights[6] = l_arm_weight;		//	LShoulderRoll
		joint_weights[7] = l_arm_weight;		//	LElbowYaw
		joint_weights[8] = l_arm_weight;		//	LElbowRoll
		joint_weights[9] = l_arm_weight;		//	LWristYaw
		joint_weights[10] = r_arm_weight;		//	RShoulderPitch
		joint_weights[11] = r_arm_weight;		//	RShoulderRoll
		joint_weights[12] = r_arm_weight;		//	RElbowYaw
		joint_weights[13] = r_arm_weight;		//	RElbowRoll
		joint_weights[14] = r_arm_weight;		//	RWristYaw
		joint_weights[15] = 0.0;				//	LHipYawPitch
		joint_weights[16] = leg_weight;			//	LHipRoll
		joint_weights[17] = leg_weight;			//	LHipPitch
		joint_weights[18] = leg_weight;			//	LKneePitch
		joint_weights[19] = leg_weight;			//	LAnklePitch
		joint_weights[20] = leg_weight;			//	LAnkleRoll
		planner_->setJointWeights(joint_weights);



        std::cout<<"Right before setStartGoalConfigs in nao_planner_control"<<std::endl;

		if (planner_->setStartGoalConfigs(start_config,goal_config))
			{
				//Solve the query
			    int num_nodes = 0;
			    float time_elapsed = 0.0;
				resp.motion_plan = planner_->solveQuery(MAX_EXPAND_ITERATIONS, ADVANCE_STEPS,SOLUTION_FILE_PATH_FIRST,num_nodes,time_elapsed);
				resp.num_nodes_generated = num_nodes;
				resp.search_time = time_elapsed;
			}


		if(resp.motion_plan.trajectory.size()>0 && resp.motion_plan.trajectory[0].joint_trajectory.points.size() > 0)
			resp.success = true;
		else
			resp.success = false;
	}
	else
	{
		resp.success = false;
	}


	return true;
}

bool NAO_PLANNER_CONTROL::compute_linear_manipulation_plan(rrt_planner_msgs::Compute_Linear_Manipulation_Plan::Request &req, rrt_planner_msgs::Compute_Linear_Manipulation_Plan::Response &resp)
{

    std::cout <<"Linear Manipulation Plan requested"<< std::endl;

//    //-----------Start Configuration (= current robot state in planning_scene)
//	//Get the joint names of the planning group
//    //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();

//	//Get the current configuration (for the whole robot)
//    //planning_scene_monitor::LockedPlanningSceneRO p_s(psm_);
//    robot_state::RobotState state = p_s_->getCurrentState();

//    //Get Joint Names
//    std::vector<std::string> joint_title;
//    joint_title = state.getVariableNames();

//    //Get Joint Values
//    const double *joint_vals;
//    joint_vals = state.getVariablePositions();

//    std::map< std::string, double > joint_values;
//    for (int i = 0; i < joint_title.size(); i++)
//    {
//        joint_values.insert ( std::pair<std::string,double>(joint_title[i],joint_vals[i]));
//    }


//    //std::map< std::string, double > joint_values;
//    //state.getStateValues(joint_values);

//	//Erase configuration elements not used for planning
//	joint_values.erase("HeadPitch");
//	joint_values.erase("HeadYaw");
//	joint_values.erase("world_joint/theta");
//	joint_values.erase("world_joint/x");
//	joint_values.erase("world_joint/y");

//	//Plot remaining configuration
//	//	for(std::map<std::string,double>::const_iterator i = joint_values.begin(); i != joint_values.end(); ++i)
//	//	{
//	//		std::cout << i->first << ": " << i->second << std::endl;
//	//	}

//	//Configuration for Planning (rearrange joint values in the order of the Kin. Model)
//    std::vector<double> start_config(joint_names_.size());
//    std::cout<<"Start Config."<<std::endl;
//    for(unsigned int i = 0 ; i < joint_names_.size(); i++)
//	{
//     start_config[i] = joint_values[joint_names_[i]];
//     std::cout << joint_names_[i] << ": " << joint_values[joint_names_[i]] << std::endl;
//	}


    //-----------Start Configuration (from request) -------------------------
    std::vector<double> start_config(joint_names_.size());
    std::cout<<"Start Config."<<std::endl;
    for(unsigned int i = 0 ; i < joint_names_.size(); i++)
    {
     start_config[i] = req.wb_start_config[i];
     std::cout << joint_names_[i] << ": " << start_config[i] << std::endl;
    }

	//-----------Init Planning variables
	//Weights for limb joints
	double r_arm_weight = 0.0;
	double l_arm_weight = 0.0;
	double leg_weight = 0.0;
	//Weight array
    std::vector<double> joint_weights(joint_names_.size());

	//Array storing the goal config for the approach motion plan
    std::vector<double> goal_config_approach(joint_names_.size());
	//Array storing the goal config for the manipulate motion plan
    std::vector<double> goal_config_manipulate(joint_names_.size());


	//Set trajectory mode (false will prepare a trajectory for simulation, true will prepare a trajectory for execution)
	planner_->setTrajectoryMode(req.execute_trajectory);

	//------------------ Motion Planning Approach
	//Goal Configuration for approach (generated by SCG)
	std::vector<double> desired_right_hand_pose(6);
	//Set the desired position of the right hand w.r.t the right foot
	desired_right_hand_pose[0] = req.right_hand_position.x;   // X [m]
	desired_right_hand_pose[1] = req.right_hand_position.y; // Y [m]
	desired_right_hand_pose[2] = req.right_hand_position.z;  // Z [m]
	//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
	desired_right_hand_pose[3] = req.right_hand_direction.x;   // X component of hand z-axis
	desired_right_hand_pose[4] = req.right_hand_direction.y; // Y component of hand z-axis
	desired_right_hand_pose[5] = req.right_hand_direction.z;  // Z component of hand z-axis

	//Sample a goal configuration
	scg_->setDesiredRightArmPose(desired_right_hand_pose);
	int num_goal_configs_found = 0;
	float time_for_first_config = 0.0;
	num_goal_configs_found = scg_->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, true, SSC_DS_GOAL_CONFIG_FIRST);
	//Time required to find first config
	resp.time_goal_config_approach = time_for_first_config;

	//Proceed only when a goal config has been found
	if (num_goal_configs_found > 0)
	{
		//Get the best goal config from the file
		scg_->loadGoalConfig(goal_config_approach, SSC_DS_GOAL_CONFIG_FIRST);

		//Assign best goal config to service response
		resp.wb_goal_config_approach = goal_config_approach;

		//Set joint weights
		r_arm_weight = R_ARM_WEIGHT_APPROACH;
		l_arm_weight = L_ARM_WEIGHT_APPROACH;
		leg_weight = LEGS_WEIGHT_APPROACH;
		joint_weights[0] = leg_weight;			//	RAnkleRoll
		joint_weights[1] = leg_weight;			//	RAnklePitch
		joint_weights[2] = leg_weight;			//	RKneePitch
		joint_weights[3] = leg_weight;			//	RHipPitch
		joint_weights[4] = leg_weight;			//	RHipRoll
		joint_weights[5] = l_arm_weight;		//	LShoulderPitch
		joint_weights[6] = l_arm_weight;		//	LShoulderRoll
		joint_weights[7] = l_arm_weight;		//	LElbowYaw
		joint_weights[8] = l_arm_weight;		//	LElbowRoll
		joint_weights[9] = l_arm_weight;		//	LWristYaw
		joint_weights[10] = r_arm_weight;		//	RShoulderPitch
		joint_weights[11] = r_arm_weight;		//	RShoulderRoll
		joint_weights[12] = r_arm_weight;		//	RElbowYaw
		joint_weights[13] = r_arm_weight;		//	RElbowRoll
		joint_weights[14] = r_arm_weight;		//	RWristYaw
		joint_weights[15] = 0.0;				//	LHipYawPitch
		joint_weights[16] = leg_weight;			//	LHipRoll
		joint_weights[17] = leg_weight;			//	LHipPitch
		joint_weights[18] = leg_weight;			//	LKneePitch
		joint_weights[19] = leg_weight;			//	LAnklePitch
		joint_weights[20] = leg_weight;			//	LAnkleRoll
		planner_->setJointWeights(joint_weights);



		if (planner_->setStartGoalConfigs(start_config,goal_config_approach))
			{
				//Solve the query
			    int num_nodes = 0;
			    float time_elapsed = 0.0;
				resp.motion_plan_approach = planner_->solveQuery(MAX_EXPAND_ITERATIONS, ADVANCE_STEPS,SOLUTION_FILE_PATH_FIRST, num_nodes,time_elapsed);
				resp.num_nodes_generated_approach = num_nodes;
				resp.search_time_approach = time_elapsed;
			}

		if( resp.motion_plan_approach.trajectory.size()>0 &&  resp.motion_plan_approach.trajectory[0].joint_trajectory.points.size() > 0)
			resp.success_approach = true;
		else
			resp.success_approach = false;

	}
	else
	{
		resp.success_approach = false;
	}


	//------------------ Motion Planning Manipulation
	if (resp.success_approach == true)
	{
		//-----------Goal Configuration for manipulation (generated by SCG)
		std::vector<double> desired_right_hand_pose_manipulate(6);
		double pos_angle = (90.0 - req.relative_angle)*(M_PI/180);
		//Set the desired position of the right hand w.r.t the right foot
		desired_right_hand_pose_manipulate[0] = desired_right_hand_pose[0] - req.object_translation_length * sin(pos_angle);   // X [m]
		desired_right_hand_pose_manipulate[1] = desired_right_hand_pose[1] + req.object_translation_length * cos(pos_angle);; // Y [m]
		desired_right_hand_pose_manipulate[2] = req.right_hand_position.z;  // Z [m]
		//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
		desired_right_hand_pose_manipulate[3] = req.right_hand_direction.x;   // X component of hand z-axis
		desired_right_hand_pose_manipulate[4] = req.right_hand_direction.y; // Y component of hand z-axis
		desired_right_hand_pose_manipulate[5] = req.right_hand_direction.z;  // Z component of hand z-axis

		//Sample a Goal Configuration
		scg_->setDesiredRightArmPose(desired_right_hand_pose_manipulate);
		time_for_first_config = 0.0;
		num_goal_configs_found = scg_->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, true, SSC_DS_GOAL_CONFIG_SECOND);
		//Time required to find first config
		resp.time_goal_config_manipulate = time_for_first_config;

		if (num_goal_configs_found > 0)
		{
			//Get the best goal config from the file
			scg_->loadGoalConfig(goal_config_manipulate, SSC_DS_GOAL_CONFIG_SECOND);

			//Assign best goal config to service response
			resp.wb_goal_config_manipulate = goal_config_manipulate;

			//Activate the manipulation constraint
			planner_->activate_drawer_constraint(desired_right_hand_pose,desired_right_hand_pose_manipulate,20,req.object_translation_length);

			//Set joint weights
			r_arm_weight = R_ARM_WEIGHT_MANIPULATE;
			l_arm_weight = L_ARM_WEIGHT_MANIPULATE;
			leg_weight = LEGS_WEIGHT_MANIPULATE;
			joint_weights[0] = leg_weight;			//	RAnkleRoll
			joint_weights[1] = leg_weight;			//	RAnklePitch
			joint_weights[2] = leg_weight;			//	RKneePitch
			joint_weights[3] = leg_weight;			//	RHipPitch
			joint_weights[4] = leg_weight;			//	RHipRoll
			joint_weights[5] = l_arm_weight;		//	LShoulderPitch
			joint_weights[6] = l_arm_weight;		//	LShoulderRoll
			joint_weights[7] = l_arm_weight;		//	LElbowYaw
			joint_weights[8] = l_arm_weight;		//	LElbowRoll
			joint_weights[9] = l_arm_weight;		//	LWristYaw
			joint_weights[10] = r_arm_weight;		//	RShoulderPitch
			joint_weights[11] = r_arm_weight;		//	RShoulderRoll
			joint_weights[12] = r_arm_weight;		//	RElbowYaw
			joint_weights[13] = r_arm_weight;		//	RElbowRoll
			joint_weights[14] = r_arm_weight;		//	RWristYaw
			joint_weights[15] = 0.0;				//	LHipYawPitch
			joint_weights[16] = leg_weight;			//	LHipRoll
			joint_weights[17] = leg_weight;			//	LHipPitch
			joint_weights[18] = leg_weight;			//	LKneePitch
			joint_weights[19] = leg_weight;			//	LAnklePitch
			joint_weights[20] = leg_weight;			//	LAnkleRoll
			planner_->setJointWeights(joint_weights);



			if (planner_->setStartGoalConfigs(goal_config_approach,goal_config_manipulate))
				{
					//Solve the query
				 	int num_nodes = 0;
				 	float time_elapsed = 0.0;
					resp.motion_plan_manipulation = planner_->solveQuery(MAX_EXPAND_ITERATIONS, ADVANCE_STEPS,SOLUTION_FILE_PATH_SECOND, num_nodes,time_elapsed);
					resp.num_nodes_generated_manipulate =  num_nodes;
					resp.search_time_manipulate =  time_elapsed;
				}

                        if( resp.motion_plan_manipulation.trajectory.size()>0 &&  resp.motion_plan_manipulation.trajectory[0].joint_trajectory.points.size() > 0)
			{
				resp.success_manipulation = true;
			}
			else
				resp.success_manipulation = false;


		}//END if num_goal_configs_found > 0

		else
		resp.success_manipulation = false;

	}//END if resp.success_approach == true



	return true;
}



bool NAO_PLANNER_CONTROL::compute_circular_manipulation_plan(rrt_planner_msgs::Compute_Circular_Manipulation_Plan::Request &req, rrt_planner_msgs::Compute_Circular_Manipulation_Plan::Response &resp)
{

    std::cout <<"Circular Manipulation Plan requested"<< std::endl;

//	//-----------Start Configuration (= current robot state in planning_scene)
//	//Get the joint names of the planning group
//    //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();


//	//Get the current configuration (for the whole robot)
//    //planning_scene_monitor::LockedPlanningSceneRO p_s(psm_);
//    robot_state::RobotState state = p_s_->getCurrentState();


//    //Get Joint Names
//    std::vector<std::string> joint_title;
//    joint_title = state.getVariableNames();

//    //Get Joint Values
//    const double *joint_vals;
//    joint_vals = state.getVariablePositions();

//    std::map< std::string, double > joint_values;
//    for (int i = 0; i < joint_title.size(); i++)
//    {
//        joint_values.insert ( std::pair<std::string,double>(joint_title[i],joint_vals[i]));
//    }


//    //std::map< std::string, double > joint_values;
//    //state.getStateValues(joint_values);

//	//Erase configuration elements not used for planning
//	joint_values.erase("HeadPitch");
//	joint_values.erase("HeadYaw");
//	joint_values.erase("world_joint/theta");
//	joint_values.erase("world_joint/x");
//	joint_values.erase("world_joint/y");

//	//Plot remaining configuration
//	//	for(std::map<std::string,double>::const_iterator i = joint_values.begin(); i != joint_values.end(); ++i)
//	//	{
//	//		std::cout << i->first << ": " << i->second << std::endl;
//	//	}

//	//Configuration for Planning (rearrange joint values in the order of the Kin. Model)
//    std::vector<double> start_config(joint_names_.size());
//    for(unsigned int i = 0 ; i < joint_names_.size(); i++)
//	{
//     start_config[i] = joint_values[joint_names_[i]];
//     //std::cout << joint_names[i] << ": " << joint_values[joint_names_[i]] << std::endl;
//	}


    //-----------Start Configuration (from request) -------------------------
    std::vector<double> start_config(joint_names_.size());
    std::cout<<"Start Config."<<std::endl;
    for(unsigned int i = 0 ; i < joint_names_.size(); i++)
    {
     start_config[i] = req.wb_start_config[i];
     std::cout << joint_names_[i] << ": " << start_config[i] << std::endl;
    }



	//-----------Init Planning variables
	//Weights for limb joints
	double r_arm_weight = 0.0;
	double l_arm_weight = 0.0;
	double leg_weight = 0.0;
	//Weight array
    std::vector<double> joint_weights(joint_names_.size());

	//Array storing the goal config for the approach motion plan
    std::vector<double> goal_config_approach(joint_names_.size());
	//Array storing the goal config for the manipulate motion plan
    std::vector<double> goal_config_manipulate(joint_names_.size());


	//Set trajectory mode (false will prepare a trajectory for simulation, true will prepare a trajectory for execution)
	planner_->setTrajectoryMode(req.execute_trajectory);

	//------------------ Motion Planning Approach
	//Goal Configuration for approach (generated by SCG)
	std::vector<double> desired_right_hand_pose(6);
	//Set the desired position of the right hand w.r.t the right foot
	desired_right_hand_pose[0] = req.right_hand_position.x;   // X [m]
	desired_right_hand_pose[1] = req.right_hand_position.y; // Y [m]
	desired_right_hand_pose[2] = req.right_hand_position.z;  // Z [m]
	//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
	desired_right_hand_pose[3] = req.right_hand_direction.x;   // X component of hand z-axis
	desired_right_hand_pose[4] = req.right_hand_direction.y; // Y component of hand z-axis
	desired_right_hand_pose[5] = req.right_hand_direction.z;  // Z component of hand z-axis

	//Sample a Goal Configuration
	scg_->setDesiredRightArmPose(desired_right_hand_pose);
	int num_goal_configs_found = 0;
	float time_for_first_config = 0.0;
	num_goal_configs_found = scg_->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, true, SSC_DS_GOAL_CONFIG_FIRST);
	//Time required to find first config
	resp.time_goal_config_approach = time_for_first_config;


	//Proceed only when a goal config has been found
	if (num_goal_configs_found > 0)
	{
		//Get the best goal config from the file
		scg_->loadGoalConfig(goal_config_approach, SSC_DS_GOAL_CONFIG_FIRST);

		//Assign best goal config to service response
		resp.wb_goal_config_approach = goal_config_approach;


		//Set joint weights
		r_arm_weight =  R_ARM_WEIGHT_APPROACH;
		l_arm_weight =  L_ARM_WEIGHT_APPROACH;
		leg_weight =  LEGS_WEIGHT_APPROACH;
		joint_weights[0] = leg_weight;			//	RAnkleRoll
		joint_weights[1] = leg_weight;			//	RAnklePitch
		joint_weights[2] = leg_weight;			//	RKneePitch
		joint_weights[3] = leg_weight;			//	RHipPitch
		joint_weights[4] = leg_weight;			//	RHipRoll
		joint_weights[5] = l_arm_weight;		//	LShoulderPitch
		joint_weights[6] = l_arm_weight;		//	LShoulderRoll
		joint_weights[7] = l_arm_weight;		//	LElbowYaw
		joint_weights[8] = l_arm_weight;		//	LElbowRoll
		joint_weights[9] = l_arm_weight;		//	LWristYaw
		joint_weights[10] = r_arm_weight;		//	RShoulderPitch
		joint_weights[11] = r_arm_weight;		//	RShoulderRoll
		joint_weights[12] = r_arm_weight;		//	RElbowYaw
		joint_weights[13] = r_arm_weight;		//	RElbowRoll
		joint_weights[14] = r_arm_weight;		//	RWristYaw
		joint_weights[15] = 0.0;				//	LHipYawPitch
		joint_weights[16] = leg_weight;			//	LHipRoll
		joint_weights[17] = leg_weight;			//	LHipPitch
		joint_weights[18] = leg_weight;			//	LKneePitch
		joint_weights[19] = leg_weight;			//	LAnklePitch
		joint_weights[20] = leg_weight;			//	LAnkleRoll
		planner_->setJointWeights(joint_weights);



		if (planner_->setStartGoalConfigs(start_config,goal_config_approach))
			{
				//Solve the query
				int num_nodes = 0;
				float time_elapsed = 0.0;
				resp.motion_plan_approach = planner_->solveQuery(MAX_EXPAND_ITERATIONS, ADVANCE_STEPS,SOLUTION_FILE_PATH_FIRST, num_nodes,time_elapsed);
				resp.num_nodes_generated_approach = num_nodes;
				resp.search_time_approach = time_elapsed;
			}

		if(resp.motion_plan_approach.trajectory.size() > 0 && resp.motion_plan_approach.trajectory[0].joint_trajectory.points.size() > 0)
			resp.success_approach = true;
		else
			resp.success_approach = false;
	}
	else
	{
		resp.success_approach = false;
	}



	//------------------ Motion Planning Manipulation
	if (resp.success_approach == true)
	{
		//Goal Configuration for manipulation (generated by SCG)
		//From handle position to point on the door
		double x_door = desired_right_hand_pose[0] + req.clearance_handle * cos(req.relative_angle*(M_PI/180)); 
		double y_door = desired_right_hand_pose[1] - req.clearance_handle * sin(req.relative_angle*(M_PI/180));
		double z_door = desired_right_hand_pose[2];
		
		//From point on the door to hinge position
		double x_hinge = x_door - req.rotation_radius * sin(req.relative_angle*(M_PI/180));
		double y_hinge = y_door - req.rotation_radius * cos(req.relative_angle*(M_PI/180));
		double z_hinge = z_door;
		
		//Goal position for point on the door
		double tmp_angle = (req.rotation_angle-req.relative_angle) *(M_PI/180);
		x_door = x_hinge - req.rotation_radius * sin(tmp_angle);   // X [m]; 
		y_door = y_hinge + req.rotation_radius * cos(tmp_angle); // Y [m];
		z_door = z_hinge;  // Z [m];	
		
		//Goal position for door handle
		tmp_angle = 1.5708 - tmp_angle; // 90 degree - tmp_angle
		std::vector<double> desired_right_hand_pose_manipulate(6);
		desired_right_hand_pose_manipulate[0] = x_door - req.clearance_handle * sin(tmp_angle);
		desired_right_hand_pose_manipulate[1] = y_door - req.clearance_handle * cos(tmp_angle);
		desired_right_hand_pose_manipulate[2] = z_door;
		//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
		desired_right_hand_pose_manipulate[3] = req.right_hand_direction.x;   // X component of hand z-axis
		desired_right_hand_pose_manipulate[4] = req.right_hand_direction.y; // Y component of hand z-axis
		desired_right_hand_pose_manipulate[5] = req.right_hand_direction.z;  // Z component of hand z-axis

		//Sample a Goal Configuration
		scg_->setDesiredRightArmPose(desired_right_hand_pose_manipulate);
		time_for_first_config = 0.0;
		num_goal_configs_found = scg_->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, true, SSC_DS_GOAL_CONFIG_SECOND);
		//Time required to find first config
		resp.time_goal_config_manipulate = time_for_first_config;

		if (num_goal_configs_found > 0)
		{
			//Get the best goal config from the file
			scg_->loadGoalConfig(goal_config_manipulate, SSC_DS_GOAL_CONFIG_SECOND);

			//Assign best goal config to service response
			resp.wb_goal_config_manipulate = goal_config_manipulate;


			//Parameters for the door (required)
			std::vector<double> object_parameter(4);
			object_parameter[0] = req.rotation_radius; 	//door width
			object_parameter[1] = req.rotation_angle;	//door opening angle
			object_parameter[2] = req.relative_angle;	//relative angle of door w.r.t right foot frame
			object_parameter[3] = req.clearance_handle;	// distance between door and handle
			//Activate manipulation constraint
			planner_->activate_door_constraint(desired_right_hand_pose,desired_right_hand_pose_manipulate,20,object_parameter);


			//Set joint weights
			r_arm_weight =  R_ARM_WEIGHT_MANIPULATE;
			l_arm_weight =  L_ARM_WEIGHT_MANIPULATE;
			leg_weight =  LEGS_WEIGHT_MANIPULATE;
			joint_weights[0] = leg_weight;			//	RAnkleRoll
			joint_weights[1] = leg_weight;			//	RAnklePitch
			joint_weights[2] = leg_weight;			//	RKneePitch
			joint_weights[3] = leg_weight;			//	RHipPitch
			joint_weights[4] = leg_weight;			//	RHipRoll
			joint_weights[5] = l_arm_weight;		//	LShoulderPitch
			joint_weights[6] = l_arm_weight;		//	LShoulderRoll
			joint_weights[7] = l_arm_weight;		//	LElbowYaw
			joint_weights[8] = l_arm_weight;		//	LElbowRoll
			joint_weights[9] = l_arm_weight;		//	LWristYaw
			joint_weights[10] = r_arm_weight;		//	RShoulderPitch
			joint_weights[11] = r_arm_weight;		//	RShoulderRoll
			joint_weights[12] = r_arm_weight;		//	RElbowYaw
			joint_weights[13] = r_arm_weight;		//	RElbowRoll
			joint_weights[14] = r_arm_weight;		//	RWristYaw
			joint_weights[15] = 0.0;				//	LHipYawPitch
			joint_weights[16] = leg_weight;			//	LHipRoll
			joint_weights[17] = leg_weight;			//	LHipPitch
			joint_weights[18] = leg_weight;			//	LKneePitch
			joint_weights[19] = leg_weight;			//	LAnklePitch
			joint_weights[20] = leg_weight;			//	LAnkleRoll
			planner_->setJointWeights(joint_weights);



			if (planner_->setStartGoalConfigs(goal_config_approach,goal_config_manipulate))
				{
					//Solve the query
					int num_nodes = 0;
					float time_elapsed = 0.0;
					resp.motion_plan_manipulation = planner_->solveQuery(MAX_EXPAND_ITERATIONS, ADVANCE_STEPS,SOLUTION_FILE_PATH_SECOND, num_nodes,time_elapsed);
					resp.num_nodes_generated_manipulate = num_nodes;
					resp.search_time_manipulate = time_elapsed;
				}

			if(resp.motion_plan_manipulation.trajectory.size() > 0 && resp.motion_plan_manipulation.trajectory[0].joint_trajectory.points.size() > 0)
			{
				resp.success_manipulation = true;
			}
			else
				resp.success_manipulation = false;

		} //END if num_goal_configs_found > 0

		else
		{
			resp.success_manipulation = false;
		}


	}//END if resp.success == true

	return true;
}



//Fix Joint name order from RobotModel to order used by planner
std::vector<std::string> NAO_PLANNER_CONTROL::joint_name_order_RobModel_to_Planner(const std::vector<std::string> j_names)
{
    std::vector<std::string> planning_j_name_order(j_names.size()-1);

    planning_j_name_order[0] = "RAnkleRoll";
    planning_j_name_order[1] = "RAnklePitch";
    planning_j_name_order[2] = "RKneePitch";
    planning_j_name_order[3] = "RHipPitch";
    planning_j_name_order[4] = "RHipRoll";
    planning_j_name_order[5] = "LShoulderPitch";
    planning_j_name_order[6] = "LShoulderRoll";
    planning_j_name_order[7] = "LElbowYaw";
    planning_j_name_order[8] = "LElbowRoll";
    planning_j_name_order[9] = "LWristYaw";
    planning_j_name_order[10] = "RShoulderPitch";
    planning_j_name_order[11] = "RShoulderRoll";
    planning_j_name_order[12] = "RElbowYaw";
    planning_j_name_order[13] = "RElbowRoll";
    planning_j_name_order[14] = "RWristYaw";
    planning_j_name_order[15] = "LHipYawPitch";
    planning_j_name_order[16] = "LHipRoll";
    planning_j_name_order[17] = "LHipPitch";
    planning_j_name_order[18] = "LKneePitch";
    planning_j_name_order[19] = "LAnklePitch";
    planning_j_name_order[20] = "LAnkleRoll";

    //Order used for....
    // PLANNING:                     ROBOT_MODEL:

    //RAnkleRoll                     LHipYawPitch
    //RAnklePitch                    LHipRoll
    //RKneePitch                     LHipPitch
    //RHipPitch                      LKneePitch
    //RHipRoll                       LAnklePitch
    //LShoulderPitch                 LAnkleRoll
    //LShoulderRoll                  LShoulderPitch
    //LElbowYaw                      LShoulderRoll
    //LElbowRoll                     LElbowYaw
    //LWristYaw                      LElbowRoll
    //RShoulderPitch                 LWristYaw
    //RShoulderRoll                  RHipYawPitch
    //RElbowYaw                      RHipRoll
    //RElbowRoll                     RHipPitch
    //RWristYaw                      RKneePitch
    //LHipYawPitch                   RAnklePitch
    //LHipRoll                       RAnkleRoll
    //LHipPitch                      RShoulderPitch
    //LKneePitch                     RShoulderRoll
    //LAnklePitch                    RElbowYaw
    //LAnkleRoll                     RElbowRoll
    //                               RWristYaw

    return planning_j_name_order;
}


}
