/*
 * nao_rrt_planner.cpp
 *
 *  Created on: May 10, 2012
 *      Author: Felix Burget
 */

#include <ros/ros.h>
#include <rrt_connect_planner/rrt_planner.h>
#include <rrt_connect_planner/stable_config_generator.h>
#include <fstream>
#include <math.h>

//-------------------- GENERAL PARAMETERS -------------------------
static const std::string ROBOT_DESCRIPTION = "robot_description";      // name of the robot description (a param name, so it can be changed externally)

static const std::string PLANNING_GROUP = "whole_body_rfoot";  // group to plan for

//Scaling factor for the support polygon (scaling to avoid projected COM to reach SP border)
//Used by both CONFIG GENERATOR and MOTION PLANNER
double SCALE_SP = 0.8;

//---------------- STABLE CONFIG GENERATOR PARAMETERS ----------------
static const int MAX_SAMPLES = 3000;
static const int MAX_IK_ITERATIONS = 5;

//Set the file containing the statically stable GOAL configurations (for the first motion plan)
char* SSC_DS_GOAL_CONFIGS_FIRST = (char *)"config_database/ssc_double_support_goal_first.dat";
//Set the file containing the statically stable GOAL configurations (for the first motion plan)
char* SOLUTION_FILE_PATH_FIRST = (char *)"solutions/solution_path_first.dat";

//Set the file containing the statically stable GOAL configurations (for the second motion plan)
char* SSC_DS_GOAL_CONFIGS_SECOND = (char *)"config_database/ssc_double_support_goal_second.dat";
//Set the file containing the statically stable GOAL configurations (for the first motion plan)
char* SOLUTION_FILE_PATH_SECOND = (char *)"solutions/solution_path_second.dat";

//Set the file containing the statically stable configurations
char* SSC_DS_CONFIGS = (char *)"config_database/ssc_double_support.dat";

//Flag to enable/disable generation of statically stable configurations
bool GENERATE_STAT_STABLE_CONFIG_DATABASE = false;

//Flag to enable/disable goal configuration generation for Motion Plan 1 (Approach object)
bool GENERATE_GOAL_CONFIGS_FIRST = true;

//Flag to enable/disable goal configuration generation for Motion Plan 2 (Manipulate object)
bool GENERATE_GOAL_CONFIGS_SECOND = false;


//----------------- RRT CONNECT PLANNER PARAMETERS -------------------
double ADVANCE_STEPS = 0.1; //0.1 optimal

int MAX_EXPAND_ITERATIONS = 15000;


//---------------------- SCENARIO FLAG --------------------------
enum Scenario{OPEN_DRAWER,OPEN_DOOR};
Scenario PLANNING_SCENARIO = OPEN_DRAWER;
//Scenario PLANNING_SCENARIO = OPEN_DOOR;






//++++++++++++++++++++++++++++++++++++++++++ MAIN ++++++++++++++++++++++++++++++++++++++++++++++++++++++
int main(int argc, char **argv)
{
  ros::init(argc, argv, "nao_rrt_planner");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Node Handle
  ros::NodeHandle nh;


  //boost::shared_ptr<tf::TransformListener> tf(new tf::TransformListener);
  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm;
  psm = boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));
  ros::Publisher scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Duration(0.5).sleep();

  //trajectory publisher
  ros::Publisher pub_traj= nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);

  // ------------------- Generate Stat.-stable configurations / Generate Goal configurations  -------------------
  //Create a Configuration Generator
  boost::shared_ptr<nao_constraint_sampler::StableConfigGenerator> scg;
  scg = boost::shared_ptr<nao_constraint_sampler::StableConfigGenerator>(new nao_constraint_sampler::StableConfigGenerator(psm, PLANNING_GROUP, SCALE_SP));


  //Create a set of statically stable configurations that can be used by the RRT sampler(no right arm IK performed)
  float time_for_first_config = 0.0;
  if (GENERATE_STAT_STABLE_CONFIG_DATABASE)
  {
	  scg->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, false, SSC_DS_CONFIGS);
  }

  // --------------------- Setup Planner objects (one for approaching and one for manipulation)  -----------------------
  //Planner for approaching and object
  boost::shared_ptr<nao_planner::RRT_Planner> planner;
  planner = boost::shared_ptr<nao_planner::RRT_Planner>(new nao_planner::RRT_Planner(psm,PLANNING_GROUP));

  //Scale support polygon for first planner
  planner->scale_support_polygon(SCALE_SP);

  //Load a set of statically stable configurations
  planner->load_DS_database(SSC_DS_CONFIGS);


  //--------------------------------- Set initial config and planning frame --------------------------------
  //Get current robot state
	planning_models::KinematicState &state = psm->getPlanningScene()->getCurrentState();

	//Get the Name of the Links and Joints of a specific group
	std::vector<std::string> link_names = psm->getPlanningScene()->getKinematicModel()->getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelNames();
	const std::vector<std::string>& joint_names = psm->getPlanningScene()->getKinematicModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();

	//Map containing joint names and values
	std::map<std::string, double> nvalues;

	for (unsigned int i = 0; i != joint_names.size() ; i++)
	{
	 nvalues[joint_names[i]] = 0.0;
	 //std::cout<<joint_names2[i] <<std::endl;
	}
	//Note: ElbowRoll angles can be only positive(right arm) negative(left arm)
	nvalues["RElbowRoll"] = 0.04;
	nvalues["LElbowRoll"] = -0.04;

	//Set current robot state
	state.setStateValues(nvalues);

	//No obstacles need to be considered when collecting stat.-stable configurations
	if (!GENERATE_STAT_STABLE_CONFIG_DATABASE)
	{
//		//Insert Box into Scene
//		psm.getPlanningScene()->setName("nao_with_box");
//		//create and position Box
//		Eigen::Affine3d t;
//		t = Eigen::Translation3d(0.2, -0.12, 0.35);
//		psm.getPlanningScene()->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.015, 1.0, 0.015)), t);
//
//		//create and position Box
//		Eigen::Affine3d s;
//		s = Eigen::Translation3d(0.17, -0.12, 0.5);
//		psm.getPlanningScene()->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.015, 1.0, 0.015)), s);

//		//create and position Box
//		Eigen::Affine3d s;
//		s = Eigen::Translation3d(0.15 + 0.25, -0.065, 0.15);
//		psm.getPlanningScene()->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.2, 0.3)), s);

//		//--------------------------- Insert Table into Scene -----------------------------------------------
		psm->getPlanningScene()->setName("nao_with_table");
		//create and position Box
		Eigen::Affine3d t;
		t = Eigen::Translation3d(0.35, 0.06, 0.35);
		psm->getPlanningScene()->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.6, 0.03)), t);
//		//--------------------------------------------------------------------------------------------
	}

	//Publish the planning scene, to trigger the change of the root joint in Rviz
	moveit_msgs::PlanningScene psmsg;
	psm->getPlanningScene()->getPlanningSceneMsg(psmsg);
	//Set planning frame to right foot
	psmsg.robot_model_root = "r_sole";
	//psmsg.robot_model_root = "r_gripper";
	scene_pub.publish(psmsg);
	sleep(1);

	//output the planning frame
	std::string planningframe;
	planningframe = psm->getPlanningScene()->getPlanningFrame();
	std::cout<< "Planning Frame: "<< planningframe << std::endl;



  if (psm->getPlanningScene()->isConfigured())
  {

	psm->startWorldGeometryMonitor();
	psm->startSceneMonitor();

//	// Wait for initScene to set an initial configuration
//	std::string dummy;
//	std::getline(std::cin, dummy);

	psm->startStateMonitor();


	// --------------------- Get the current robot state = Start Configuration -------------------------------------
	//Get the joint names of the planning group
	//const std::vector<std::string>& joint_names = psm->getPlanningScene()->getKinematicModel()->getJointModelGroup(PLANNING_GROUP)->getJointModelNames();


	//Get the current configuration (for the whole robot
	planning_models::KinematicState state = psm->getPlanningScene()->getCurrentState();
	std::map< std::string, double > joint_values;
	state.getStateValues(joint_values);



	//Erase configuration elements not used for planning
	joint_values.erase("HeadPitch");
	joint_values.erase("HeadYaw");
	joint_values.erase("world_joint/theta");
	joint_values.erase("world_joint/x");
	joint_values.erase("world_joint/y");

	//Plot remaining configuration
//	for(std::map<std::string,double>::const_iterator i = joint_values.begin(); i != joint_values.end(); ++i)
//	{
//		std::cout << i->first << ": " << i->second << std::endl;
//	}

	//Configuration for Planning (rearrange joint values in the order of the Kin. Model)
	std::vector<double> start_config(joint_names.size());
	for(unsigned int i = 0 ; i < joint_names.size(); i++)
	{
	 start_config[i] = joint_values[joint_names[i]];
	 //std::cout << joint_names[i] << ": " << joint_values[joint_names[i]] << std::endl;
	}



	// ------------------------Generate Goal configurations for motion plan 1 ---------------------------------
	//Set desired cartesian hand position (w.r.t right foot) and the arm reference configuration used to find best IK solution
	std::vector<double> desired_right_hand_pose(6);
	std::vector<double> r_arm_ref_config(5);
	if (PLANNING_SCENARIO == OPEN_DRAWER)
	{
		//Set the desired position of the right hand w.r.t the right foot
		desired_right_hand_pose[0] = 0.2;   // X [m]	//0.16
		desired_right_hand_pose[1] = -0.065; // Y [m]
		desired_right_hand_pose[2] = 0.2;  // Z [m]
		//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
		desired_right_hand_pose[3] = 0.0;   // X component of hand z-axis
		desired_right_hand_pose[4] = -1.0; // Y component of hand z-axis
		desired_right_hand_pose[5] = 0.0;  // Z component of hand z-axis
	}

	else if (PLANNING_SCENARIO == OPEN_DOOR)
	{
		//Set the desired position of the right hand w.r.t the right foot
		desired_right_hand_pose[0] = 0.27;   // X [m]	//0.16
		desired_right_hand_pose[1] = 0.02; // Y [m]
		desired_right_hand_pose[2] = 0.28;  // Z [m]
		//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
		desired_right_hand_pose[3] = 0.0;   // X component of hand z-axis
		desired_right_hand_pose[4] = 0.0; // Y component of hand z-axis
		desired_right_hand_pose[5] = -1.0;  // Z component of hand z-axis
	}

	else
		std::cout<< "No valid scenario chosen!"<<std::endl;

	//Generate goal configuration for first motion plan
	if(GENERATE_GOAL_CONFIGS_FIRST == true)
	{
		//Sample goal configurations for first motion (approaching)
		time_for_first_config = 0.0;
		scg->setDesiredRightArmPose(desired_right_hand_pose);
		scg->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, true, SSC_DS_GOAL_CONFIGS_FIRST);
	}


	// +++++++++++++++++++++++++++++ Motion Planning 1 (Approach/Grasp Object)+++++++++++++++++++++++++++++++++++++
	//Get a goal state from file = Goal Configuration
	//Get a random goal configuration from file
	std::vector<double> goal_config(joint_names.size());
	scg->loadGoalConfig(goal_config, SSC_DS_GOAL_CONFIGS_FIRST);

	//Set joint weights
	std::vector<double> joint_weights(joint_names.size());
	double arm_weight = 2.0;
	double leg_weight = 1.0;
	joint_weights[0] = leg_weight;				//	RAnkleRoll
	joint_weights[1] = leg_weight;				//	RAnklePitch
	joint_weights[2] = leg_weight;				//	RKneePitch
	joint_weights[3] = leg_weight;				//	RHipPitch
	joint_weights[4] = leg_weight;				//	RHipRoll
	joint_weights[5] = arm_weight;				//	LShoulderPitch
	joint_weights[6] = arm_weight;				//	LShoulderRoll
	joint_weights[7] = arm_weight;				//	LElbowYaw
	joint_weights[8] = arm_weight;				//	LElbowRoll
	joint_weights[9] = arm_weight;				//	LWristYaw
	joint_weights[10] = arm_weight;			//	RShoulderPitch
	joint_weights[11] = arm_weight;			//	RShoulderRoll
	joint_weights[12] = arm_weight;			//	RElbowYaw
	joint_weights[13] = arm_weight;			//	RElbowRoll
	joint_weights[14] = arm_weight;			//	RWristYaw
	joint_weights[15] = 0.0;			//	LHipYawPitch
	joint_weights[16] = leg_weight;			//	LHipRoll
	joint_weights[17] = leg_weight;			//	LHipPitch
	joint_weights[18] = leg_weight;			//	LKneePitch
	joint_weights[19] = leg_weight;			//	LAnklePitch
	joint_weights[20] = leg_weight;			//	LAnkleRoll
	//pass weights to function
	planner->setJointWeights(joint_weights);


	//Set the start and goal configuration (returns true if both configurations are valid)
	bool approaching_succeeded = false;
	if (planner->setStartGoalConfigs(start_config,goal_config))
	{
		//Solve the first query
		moveit_msgs::DisplayTrajectory d;
		int num_nodes = 0;
		float time_elapsed = 0.0;
		d = planner->solveQuery(MAX_EXPAND_ITERATIONS, ADVANCE_STEPS, SOLUTION_FILE_PATH_FIRST,num_nodes,time_elapsed);

		//Time for planning
		std::cout<< time_elapsed<<" seconds for planning approach"<<std::endl;


		sleep(1); //in order to give the publisher enough time before thread is killed
		pub_traj.publish(d);
		std::cout<< "Solution Trajectory published !!!"<< std::endl;
		sleep(1); //in order to give the publisher enough time before thread is killed

		std::cout<< "First Query solved, now playing the trajectory"<< std::endl;
		approaching_succeeded = true;
		//planner->playTrajectory(SOLUTION_FILE_PATH_FIRST);
	}


	// +++++++++++++++++++++++++++++++++++ Motion Planning 2 (Move Object)+++++++++++++++++++++++++++++++++++++
	// ------------------------Generate Goal configurations for motion plan 2 ---------------------------------
	std::vector<double> desired_right_hand_pose_manip(6);
	if (PLANNING_SCENARIO == OPEN_DRAWER)
	{
		//Set the desired position of the right hand w.r.t the right foot
		desired_right_hand_pose_manip[0] = desired_right_hand_pose[0] - 0.10;   // X [m]
		desired_right_hand_pose_manip[1] = desired_right_hand_pose[1]; // Y [m]
		desired_right_hand_pose_manip[2] = desired_right_hand_pose[2];  // Z [m]
		//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
		desired_right_hand_pose_manip[3] = desired_right_hand_pose[3];   // X component of hand z-axis
		desired_right_hand_pose_manip[4] = desired_right_hand_pose[4]; // Y component of hand z-axis
		desired_right_hand_pose_manip[5] = desired_right_hand_pose[5];  // Z component of hand z-axis

		planner->activate_drawer_constraint(desired_right_hand_pose,desired_right_hand_pose_manip,20, 0.1);

	}
	else if (PLANNING_SCENARIO == OPEN_DOOR)
	{
		//Parameters for the droor (required)
		std::vector<double> door_parameter(4);
		door_parameter[0] = 0.25; //radius/width of the door hinge (in meter)
		door_parameter[1] = 34.0; //rotation around door hinge (in degrees)
		door_parameter[2] = 0.0; //relative angle of door w.r.t right foot
		door_parameter[3] = 0.05;// distance between door and handle

		//Hinge position
		double x_hinge = desired_right_hand_pose[0] - door_parameter[0] * sin(door_parameter[2]*(M_PI/180));
		double y_hinge = desired_right_hand_pose[1] - door_parameter[0] * cos(door_parameter[2]*(M_PI/180));
		double z_hinge = desired_right_hand_pose[2];
		double tmp_angle = (door_parameter[1] - door_parameter[2]) *(M_PI/180);
		//Set the desired position of the right hand w.r.t the right foot
		desired_right_hand_pose_manip[0] = x_hinge - door_parameter[0] * sin(tmp_angle);   // X [m]
		desired_right_hand_pose_manip[1] = y_hinge + door_parameter[0] * cos(tmp_angle); // Y [m]
		desired_right_hand_pose_manip[2] = z_hinge;  // Z [m]
		//Set the desired direction of the z-axis of the right hand w.r.t the right foot frame
		desired_right_hand_pose_manip[3] = desired_right_hand_pose[3];   // X component of hand z-axis
		desired_right_hand_pose_manip[4] = desired_right_hand_pose[4]; // Y component of hand z-axis
		desired_right_hand_pose_manip[5] = desired_right_hand_pose[5];  // Z component of hand z-axis

		planner->activate_door_constraint(desired_right_hand_pose,desired_right_hand_pose_manip,20,door_parameter);

	}
	else
	std::cout<< "No valid scenario chosen!"<<std::endl;


	//Search a motion plan for object manipulation only when the first/previous query has been solved
	if (approaching_succeeded == true)
	{
		//Generate goal configuration for second motion plan
		if(GENERATE_GOAL_CONFIGS_SECOND == true)
		{
			//Sample goal configurations for second motion (manipulation)
			time_for_first_config = 0.0;
			scg->setDesiredRightArmPose(desired_right_hand_pose_manip);
			scg->sample_stable_configs(MAX_SAMPLES,MAX_IK_ITERATIONS,time_for_first_config, true, SSC_DS_GOAL_CONFIGS_SECOND);
		}

		//Set joint weights
		double r_arm_weight = 1.0;
		double l_arm_weight = 0.0;
		leg_weight = 0.8;
		joint_weights[0] = leg_weight;				//	RAnkleRoll
		joint_weights[1] = leg_weight;				//	RAnklePitch
		joint_weights[2] = leg_weight;				//	RKneePitch
		joint_weights[3] = leg_weight;				//	RHipPitch
		joint_weights[4] = leg_weight;				//	RHipRoll
		joint_weights[5] = l_arm_weight;				//	LShoulderPitch
		joint_weights[6] = l_arm_weight;				//	LShoulderRoll
		joint_weights[7] = l_arm_weight;				//	LElbowYaw
		joint_weights[8] = l_arm_weight;				//	LElbowRoll
		joint_weights[9] = l_arm_weight;				//	LWristYaw
		joint_weights[10] = r_arm_weight;			//	RShoulderPitch
		joint_weights[11] = r_arm_weight;			//	RShoulderRoll
		joint_weights[12] = r_arm_weight;			//	RElbowYaw
		joint_weights[13] = r_arm_weight;			//	RElbowRoll
		joint_weights[14] = r_arm_weight;			//	RWristYaw
		joint_weights[15] = 0.0;			//	LHipYawPitch
		joint_weights[16] = leg_weight;			//	LHipRoll
		joint_weights[17] = leg_weight;			//	LHipPitch
		joint_weights[18] = leg_weight;			//	LKneePitch
		joint_weights[19] = leg_weight;			//	LAnklePitch
		joint_weights[20] = leg_weight;			//	LAnkleRoll
		planner->setJointWeights(joint_weights);

		//Get goal configuration from file
		std::vector<double> goal_config_manipulation(joint_names.size());
		scg->loadGoalConfig(goal_config_manipulation, SSC_DS_GOAL_CONFIGS_SECOND);

	   //Set the start and goal configuration (returns true if both configurations are valid)
		if (planner->setStartGoalConfigs(goal_config,goal_config_manipulation))
		{
			//Solve the second query
			moveit_msgs::DisplayTrajectory d;
			int num_nodes = 0;
			float time_elapsed = 0.0;
			d = planner->solveQuery(MAX_EXPAND_ITERATIONS, ADVANCE_STEPS,SOLUTION_FILE_PATH_SECOND, num_nodes,time_elapsed);

			//Time for planning
			std::cout<< time_elapsed<<" seconds for planning manipulate"<<std::endl;

			sleep(1); //in order to give the publisher enough time before thread is killed
			pub_traj.publish(d);
			std::cout<< "Solution Trajectory published !!!"<< std::endl;
			sleep(1); //in order to give the publisher enough time before thread is killed
		}

	} //END of if(approaching_succeeded)


	//Users menu
	//	if (approaching_succeeded == true && manipulation_succeeded == true)
	//	{
	//		std::string input;
	//		std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
	//		std::getline (std::cin, input);
	//		char first_element = input[0];
	//
	//		while (first_element == 'r')
	//		{
	//			planner->playTrajectory(SOLUTION_FILE_PATH_FIRST);
	//			if (approaching_succeeded == true)
	//			planner->playTrajectory(SOLUTION_FILE_PATH_SECOND);
	//
	//			std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
	//			std::getline (std::cin, input);
	//			first_element = input[0];
	//		}
	//	}


	ros::shutdown();

    //ros::waitForShutdown();
  }


  else
    ROS_ERROR("Planning scene not configured");

  return 0;

} 
