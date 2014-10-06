/*
 * articulated_object_constraints.cpp
 *
 *  Created on: Jul 17, 2012
 *      Author: Felix Burget
 */

#include "rrt_connect_planner/articulated_object_constraints.h"
#include <math.h>

#include "ikfast_TranslationDirection5D.cpp"

namespace nao_planner {


//Constructor
ArticulatedObjectConstraints::ArticulatedObjectConstraints()
{

	//Init hand pose for start and goal tree
	pose_hand_start_tree_.resize(6);
	pose_hand_goal_tree_.resize(6);

	for (int i = 0 ; i < 6; i++)
	{
		pose_hand_start_tree_[i] = 0.0;
		pose_hand_goal_tree_[i] = 0.0;
	}


	//Definition of a kinematic chain for right leg
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.0,0.04519))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1029))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));


	//Definition of a kinematic chain for right foot to right arm
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.0,0.04519))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1029))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));
	//from here right arm joints
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.0,0.182)))); //0.182->measured by hand(not given in technical drawing of NAO)
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,-0.098,0.0))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.105,-0.015,0.0))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.05595,0.0,0.0))));
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.05775,0.0,-0.01231))));
	//Offset to grasp frame (required to make KDL and OpenRave endeffector frame identical)
	chain_r_leg_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RPY(M_PI_2,0.0,0.0),KDL::Vector(0.01,0.0,0.0))));
}


//Destructor
ArticulatedObjectConstraints::~ArticulatedObjectConstraints() {
	// TODO Auto-generated destructor stub
}



//Add a constraint for articulated object manipulation
void ArticulatedObjectConstraints::add_drawer_constraint(std::vector<double> pose_hand_start,std::vector<double> pose_hand_goal, int num_interp_points)
{
	if (pose_hand_start.size() < 6 || pose_hand_goal.size() < 6 )
		std::cout<< "ERROR: pose_hand_start or pose_hand_goal not fully specified"<<std::endl;


	else
	{
		//Set initial hand pose for start and goal tree (x,y,z,Xz,Yz,Zz)
		for (int i = 0 ; i < 6; i++)
		{
			pose_hand_start_tree_[i] = pose_hand_start[i];
			pose_hand_goal_tree_[i] = pose_hand_goal[i];
		}

		//Set the number of trajectory points
		num_intermediate_points_ = num_interp_points;

		//Set the vector array to the proper size (num_intermediate_points + start and goal hand pose * 6 values)
		hand_trajectory_points_ = new double*[num_interp_points + 2]; //creates a new array of pointers to double objects
		for(int i = 0; i < num_interp_points + 2 ; ++i)
		hand_trajectory_points_[i] = new double[6];

		//Compute num_intermediate_points_ points on the trajectory
		computeDrawerTrajectoryPoints();
	}


}

void ArticulatedObjectConstraints::add_door_constraint(std::vector<double> pose_hand_start,std::vector<double> pose_hand_goal, int num_interp_points, std::vector<double> door_param)
{
	if (pose_hand_start.size() < 6 || pose_hand_goal.size() < 6 )
			std::cout<< "ERROR: pose_hand_start or pose_hand_goal not fully specified"<<std::endl;


	else
	{
		//Set initial hand pose for start and goal tree (x,y,z,Xz,Yz,Zz)
		for (int i = 0 ; i < 6; i++)
		{
			pose_hand_start_tree_[i] = pose_hand_start[i];
			pose_hand_goal_tree_[i] = pose_hand_goal[i];
		}

		//Set the number of trajectory points
		num_intermediate_points_ = num_interp_points;

		//Set the vector array to the proper size (num_intermediate_points + start and goal hand pose * 6 values)
		hand_trajectory_points_ = new double*[num_interp_points + 2]; //creates a new array of pointers to double objects
		for(int i = 0; i < num_interp_points + 2 ; ++i)
		hand_trajectory_points_[i] = new double[6];

		//Compute num_intermediate_points_ points on the trajectory
		computeDoorTrajectoryPoints(door_param);
	}
}

void ArticulatedObjectConstraints::computeDrawerTrajectoryPoints()
{

		//Compute the step size
		std::vector<double> step(6);
		for (int i = 0 ; i < 6; i++)
		{
			//From start to goal we need to perform (num_intermediate_points_ + 1) steps
			step[i] = (pose_hand_goal_tree_[i]-pose_hand_start_tree_[i]) / (num_intermediate_points_ + 1) ;
		}



		//Stepping from start to goal and generating intermediate points
		for (int s = 0 ; s < num_intermediate_points_ + 2; s++)
		{
			for (int i = 0 ; i < 6; i++)
			{
				hand_trajectory_points_[s][i] = pose_hand_start_tree_[i] + (step[i]*s);
				//std::cout<<hand_trajectory_points_[s][i]<< " ";
			}
			//std::cout<<std::endl;
		}
}


void ArticulatedObjectConstraints::computeDoorTrajectoryPoints(std::vector<double> door_param)
{
	//door_param[0] = door_width in [m]
	//door_param[1] = rotation angle in [degrees]
	//door_param[2] = relative angle in [degrees]
	//door_param[3] = distance between door and handle [m]

	//Relative door angle (w.r.t right foot frame) in rad
	double relative_angle = door_param[2]*(M_PI/180);

	//Compute the angle step size for the hand positions(door_param[1] = open angle)
	double step_angle = (door_param[1]*(M_PI/180)) / (num_intermediate_points_ + 1);

	//Compute the step size for the hand directions
	std::vector<double> step_direction(3);
	step_direction[0] = (pose_hand_goal_tree_[3]-pose_hand_start_tree_[3]) / (num_intermediate_points_ + 1) ;
	step_direction[1] = (pose_hand_goal_tree_[4]-pose_hand_start_tree_[4]) / (num_intermediate_points_ + 1) ;
	step_direction[2] = (pose_hand_goal_tree_[5]-pose_hand_start_tree_[5]) / (num_intermediate_points_ + 1) ;


	//From handle position to point on the door
	double x_door = pose_hand_start_tree_[0] + door_param[3] * cos(relative_angle); 
	double y_door = pose_hand_start_tree_[1] - door_param[3] * sin(relative_angle);
	double z_door = pose_hand_start_tree_[2];

	//Hinge position
	double x_hinge =  x_door - door_param[0] * sin(relative_angle);
	double y_hinge =  y_door - door_param[0] * cos(relative_angle);
	double z_hinge =  z_door;
	

	//Stepping from start to goal and generating intermediate points
	double tmp_angle = 0.0;
	for (int s = 0 ; s < num_intermediate_points_ + 2; s++)
	{
		//Angle of intermediate trajectory point in rad
		tmp_angle = ((step_angle*s) - relative_angle);

		//Position of point on the door 
		x_door = x_hinge - door_param[0] * sin(tmp_angle);
		y_door = y_hinge + door_param[0] * cos(tmp_angle);
		z_door = z_hinge;

		//Handle x,y,z coordinate of intermediate point
		tmp_angle = 1.5708 - tmp_angle;
		hand_trajectory_points_[s][0] = x_door - door_param[3] * sin(tmp_angle);
		hand_trajectory_points_[s][1] = y_door - door_param[3] * cos(tmp_angle);
		hand_trajectory_points_[s][2] = z_door;

		//Xz direction
		hand_trajectory_points_[s][3] = pose_hand_start_tree_[3] + (step_direction[0]*s);
		//Yz direction
		hand_trajectory_points_[s][4] = pose_hand_start_tree_[4] + (step_direction[1]*s);
		//Zz direction
		hand_trajectory_points_[s][5] = pose_hand_start_tree_[5] + (step_direction[2]*s);

//			std::cout<<hand_trajectory_points_[s][0]<< " ";
//			std::cout<<hand_trajectory_points_[s][1]<< " ";
//			std::cout<<hand_trajectory_points_[s][2]<< " ";
//			std::cout<<hand_trajectory_points_[s][3]<< " ";
//			std::cout<<hand_trajectory_points_[s][4]<< " ";
//			std::cout<<hand_trajectory_points_[s][5]<< " ";
//			std::cout<<std::endl;
	}
}


int ArticulatedObjectConstraints::get_num_trajectory_points()
{
	//num_intermediate_points + start and goal cartesian position
	return (num_intermediate_points_ + 2);
}


bool ArticulatedObjectConstraints::right_hand_at_pose(int des_cart_hand_pose_index, std::vector<double> r_leg_config, std::vector<double> r_arm_config)
{

//	std::cout <<"X_des: " <<hand_trajectory_points_[des_cart_hand_pose_index][0] <<std::endl;
//	std::cout <<"Y_des: " <<hand_trajectory_points_[des_cart_hand_pose_index][1] <<std::endl;
//	std::cout <<"Z_des: " <<hand_trajectory_points_[des_cart_hand_pose_index][2] <<std::endl;

	// --------------------------------- FK right leg foot to hand chain ---------------------------------------------
	// Create solver based on kinematic chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain_r_leg_arm_);

	// Create joint array
	unsigned int nj = chain_r_leg_arm_.getNrOfJoints();
	KDL::JntArray jointpositions = KDL::JntArray(nj);

	// Assign values to the joint positions array
	for(unsigned int i = 0; i < r_leg_config.size() ; i++){
		jointpositions(i)= r_leg_config[i];
	}
	for(unsigned int i = 0; i < r_arm_config.size() ; i++){
			jointpositions(i+r_leg_config.size())= r_arm_config[i];
	}

	//Hand frame
	KDL::Frame r_hand_frame;

	// Calculate forward kinematics (right hand pose in right foot frame)
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions,r_hand_frame);
	if(kinematics_status>=0)
	{
		//std::cout << "FK solution for right foot to arm chain:"<< std::endl << r_hand_frame <<std::endl;
	}
	else{
		printf("%s \n","ERROR: failed to calculate forward kinematics for right foot to right arm chain:(");
	}


	//----------------- Compare current right hand cartesian pose with desired cartesian pose on trajectory-----------
	std::vector<double> pose_error(6);
	//Position error
	KDL::Vector pos_hand = r_hand_frame.p;
	pose_error[0] = fabs(pos_hand.x() - hand_trajectory_points_[des_cart_hand_pose_index][0]); //X error
	pose_error[1] = fabs(pos_hand.y() - hand_trajectory_points_[des_cart_hand_pose_index][1]); //Y error
	pose_error[2] = fabs(pos_hand.z() - hand_trajectory_points_[des_cart_hand_pose_index][2]); //Z error
	//Direction error (Compute error between current and desired direction of grasp frame's z-axis w.r.t r_foot frame)
	pose_error[3] = fabs(r_hand_frame.operator()(0,2) - hand_trajectory_points_[des_cart_hand_pose_index][3]);
	pose_error[4] = fabs(r_hand_frame.operator()(1,2) - hand_trajectory_points_[des_cart_hand_pose_index][4]);
	pose_error[5] = fabs(r_hand_frame.operator()(2,2) - hand_trajectory_points_[des_cart_hand_pose_index][5]);

	//Find max position error
	double max_pos_error = 0.0;
	for (int i = 0; i < 3 ; i++)
	{
		if(pose_error[i] > max_pos_error)
			max_pos_error = pose_error[i];
	}

	//Find max direction error (in x,y,z component of grasp frame w.r.t r_foot frame)
	double max_direction_error = 0.0;
	for (int i = 3; i < 6 ; i++)
	{
		if(pose_error[i] > max_direction_error)
			max_direction_error = pose_error[i];
	}


	//Output errors
//	std::cout<<"Errors: "<<std::endl;
//	std::cout<<max_pos_error<<std::endl;
//	std::cout<<max_direction_error<<std::endl;


	if(max_pos_error > 0.005)
		return false;

	if(max_direction_error > 0.007)
		return false;

	else
	return true;

}



//bool ArticulatedObjectConstraints::compute_right_arm_IK(int des_hand_pose_index, std::vector<double> r_leg_config, std::vector<double> r_arm_config_reference, std::vector< std::pair< double, double > >  bounds_right_arm, std::vector<double>& new_r_arm_config)
bool ArticulatedObjectConstraints::compute_right_arm_IK(int des_hand_pose_index, std::vector<double> r_leg_config, std::vector<double> r_arm_config_reference, std::vector<moveit::core::VariableBounds> bounds_right_arm, std::vector<double>& new_r_arm_config)
{
	//Get the pose of the Hip Frame
	KDL::Frame hip_transform;
	getHipPose(r_leg_config,hip_transform);


//	std::cout <<"hand pose index: " <<des_hand_pose_index <<std::endl;
//	std::cout <<"X_des: " <<hand_trajectory_points_[des_hand_pose_index][0] <<std::endl;
//	std::cout <<"Y_des: " <<hand_trajectory_points_[des_hand_pose_index][1] <<std::endl;
//	std::cout <<"Z_des: " <<hand_trajectory_points_[des_hand_pose_index][2] <<std::endl;
//	std::cout <<"Xz_des: " <<hand_trajectory_points_[des_hand_pose_index][3] <<std::endl;
//	std::cout <<"Yz_des: " <<hand_trajectory_points_[des_hand_pose_index][4] <<std::endl;
//	std::cout <<"Zz_des: " <<hand_trajectory_points_[des_hand_pose_index][5] <<std::endl;

	//+++ Compute Position of right Hand in Torso frame +++
	KDL::Vector pos_r_hand_desired(hand_trajectory_points_[des_hand_pose_index][0], hand_trajectory_points_[des_hand_pose_index][1] , hand_trajectory_points_[des_hand_pose_index][2]);
	//Get the position vector right foot to hip
	KDL::Vector pos_hip = hip_transform.p;


	//Rotation from hip frame to right foot frame
	std::vector<double> hip_pose_transpose(9);
	int element = 0;
	for (unsigned int i=0;i<3;i++)
	{
		for (unsigned int j=0;j<3;j++)
		{
			hip_pose_transpose[element] = hip_transform.operator()(j,i);
			//std::cout <<"Element " << element << " of m_elements is : "<< m_elements[element] << std::endl;
			element++;
		}
	}


	//Compute the transpose rotation (r_hand frame only translated w.r.t r_foot frame)
	KDL::Rotation rotation (hip_pose_transpose[0],hip_pose_transpose[1],hip_pose_transpose[2],hip_pose_transpose[3],
			hip_pose_transpose[4],hip_pose_transpose[5],hip_pose_transpose[6],hip_pose_transpose[7],hip_pose_transpose[8]);




	//Compute desired position of right hand in hip frame (page 56 robotics handbook)
	std::vector<double> pos_r_hand_from_hip(3);
	double tmp= 0.0, tmp2 = 0.0;
	for (unsigned int i = 0 ; i < 3 ; i++)
			{
				 tmp = -rotation.operator()(i,0) * pos_hip.x() - rotation.operator()(i,1) * pos_hip.y() - rotation.operator()(i,2) * pos_hip.z();
				 tmp2 = rotation.operator()(i,0) * pos_r_hand_desired.x() + rotation.operator()(i,1) * pos_r_hand_desired.y() + rotation.operator()(i,2) * pos_r_hand_desired.z();
				 pos_r_hand_from_hip[i] = tmp + tmp2;
			}


	//Compute desired position of right hand in torso frame (simply subtract the offset in z-direction between hip and torso -> 85 mm)
	std::vector<double> pos_r_hand_from_torso(3);
	pos_r_hand_from_torso[0] =  pos_r_hand_from_hip[0];
	pos_r_hand_from_torso[1] =  pos_r_hand_from_hip[1];
	pos_r_hand_from_torso[2] = pos_r_hand_from_hip[2] - 0.085;


	// +++ Compute Direction of right Hand in Torso frame +++
	//Compute desired direction of right hand in torso frame
	std::vector<double> direction_r_hand_from_torso(3);
	for (unsigned int i = 0 ; i < 3 ; i++)
			{
			 direction_r_hand_from_torso[i] = rotation.operator()(i,0) * hand_trajectory_points_[des_hand_pose_index][3] + rotation.operator()(i,1) * hand_trajectory_points_[des_hand_pose_index][4] + rotation.operator()(i,2) * hand_trajectory_points_[des_hand_pose_index][5];
			}


//	std::cout <<"X_des: " <<pos_r_hand_from_torso[0] <<std::endl;
//	std::cout <<"Y_des: " <<pos_r_hand_from_torso[1] <<std::endl;
//	std::cout <<"Z_des: " <<pos_r_hand_from_torso[2] <<std::endl;
//	std::cout <<"Xz_des: " <<direction_r_hand_from_torso[0] <<std::endl;
//	std::cout <<"Yz_des: " <<direction_r_hand_from_torso[1] <<std::endl;
//	std::cout <<"Zz_des: " <<direction_r_hand_from_torso[2] <<std::endl;


	// ------------------------------------------- Solve IK with openRave -------------------------------------------------
	//position
	IKReal position[3];
	position[0] = pos_r_hand_from_torso[0];
	position[1] = pos_r_hand_from_torso[1];
	position[2] = pos_r_hand_from_torso[2];
	IKReal *pos = position;
	//direction
	IKReal direction[9]; //First three values define target direction
	direction[0] = direction_r_hand_from_torso[0];
	direction[1] = direction_r_hand_from_torso[1];
	direction[2] = direction_r_hand_from_torso[2];
	direction[3] = 0.0;
	direction[4] = 0.0;
	direction[5] = 0.0;
	direction[6] = 0.0;
	direction[7] = 0.0;
	direction[8] = 0.0;
	IKReal *dir = direction;

	//free joints (no free joints)
	IKReal pfree_values[5];
	pfree_values[0] = 5.0;
	pfree_values[1] = 1.0;
	pfree_values[2] = 1.0;
	pfree_values[3] = 1.0;
	pfree_values[4] = 1.0;
	IKReal *pfree = pfree_values;

	//Vector storing all Solutions found
	std::vector<IKSolution> solutions;


	//create an IK solver
	IKSolver arm_ik_solver;
	//Solve the IK
	if(arm_ik_solver.ik(pos,dir,pfree,solutions))
	{
		//Number of solutions found
		//std::cout<< "Number of solutions found: " << solutions.size()<<std::endl;


		//Number of Joints
		int num_joints = getNumJoints();



		//Init Loop parameters
		std::vector<IKReal> sol(num_joints);
		bool valid_solution = false;
		KDL::JntArray joint_values = KDL::JntArray(num_joints);

		//Minimum distance to reference right arm pose found so far
		double min_distance_to_q_ref = 10000.0;
		//Distance of current to reference right arm pose
		double current_distance_to_q_ref = 0.0;
		//Index of the closest right arm pose found so far
		int index_q_closest = 1000;



		//Iterate through solutions and try to find a valid one (joint limits respected)
		for (unsigned int solution = 0 ; solution < solutions.size() ; solution++)
		{
			//std::cout<< "Solution num: " << solution << std::endl;
			std::vector<IKReal> vsolfree(solutions[solution].GetFree().size());
			solutions[solution].GetSolution(&sol[0],vsolfree.size() > 0 ? &vsolfree[solution]:NULL);


			 //Store IK solution in KDL::JntArray
			 for (int i = 0 ; i < num_joints ; i++)
				 joint_values(i) = sol[i];

			 //Check if IK solution respects the joint limits
			 valid_solution = check_joint_limits(joint_values,bounds_right_arm);

			//If solution respects joint limits
			 if (valid_solution == true)
			 {
				 //Compute the distance between the current right arm IK solution and the reference configuration
				 current_distance_to_q_ref = compute_config_distance(joint_values, r_arm_config_reference);

				 //If the distance is smaller than the min_distance found so far.......
				 if (current_distance_to_q_ref < min_distance_to_q_ref)
				 {
					 min_distance_to_q_ref = current_distance_to_q_ref;
					 index_q_closest = solution;
				 }
			 }

		}//End of IK Solution loop


		//When a valid IK solution has been found
		if (min_distance_to_q_ref < 1000)
		{
			//Get the best IK solution found
			std::vector<IKReal> vsolfree(solutions[index_q_closest].GetFree().size());
			solutions[index_q_closest].GetSolution(&sol[0],vsolfree.size() > 0 ? &vsolfree[index_q_closest]:NULL);

			//Assign the IK solution to the return vector
			for(unsigned int i = 0 ; i < sol.size() ; i++)
			{
				//Assign IK solution to referenced left_leg_config array
				new_r_arm_config[i] = sol[i];
				//std::cout << right_arm_config[i] << std::endl;
			}

			return true;
		}

		else
		{
			//No valid solution has been found
			return false;
		}

	} //END of "if" section


	//No IK solution has been found
	else
	{
		return false;
	}

}


//Compute the pose of the hip given the configuration of the right foot (r_foot fixed on the floor)
bool ArticulatedObjectConstraints::getHipPose(std::vector<double> right_leg_config, KDL::Frame &hip_pose)
{
	// ------------------------------------------------FK right leg chain ---------------------------------------------
	// Create solver based on kinematic chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain_right_leg_);

	// Create joint array
	unsigned int nj = chain_right_leg_.getNrOfJoints();
	KDL::JntArray jointpositions = KDL::JntArray(nj);


	// Assign values to the joint positions array
	for(unsigned int i=0;i<nj;i++){
		jointpositions(i)= right_leg_config[i];
	}

	// Calculate forward position kinematics (pose of hip)
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions,hip_pose);
	 //std::cout << "FK solution for right leg:  "<< std::endl;
	if(kinematics_status>=0)
	{
		//std::cout << "FK for right leg (Pose of Hip)"<< std::endl << hip_pose <<std::endl;
	}
	else{
		printf("%s \n","ERROR: failed to calculate forward kinematics for right leg:(");
	}

	return kinematics_status;
}



//Check if a certain configuration of the left leg respects the joint limits
//bool ArticulatedObjectConstraints::check_joint_limits(KDL::JntArray ik_solution, std::vector< std::pair< double, double > > chain_bounds)
bool ArticulatedObjectConstraints::check_joint_limits(KDL::JntArray ik_solution, std::vector<moveit::core::VariableBounds> chain_bounds)
{
	bool valid = true;
	 for(unsigned int i=0 ; i<chain_bounds.size() ; i++)
	 {
		 //Lower bound is smaller than upper bound
         if (chain_bounds[i].min_position_ < chain_bounds[i].max_position_)
		 {
			 //check if joint value is above the lower joint limit
             if (ik_solution(i) > chain_bounds[i].min_position_)
			 {
				//check if joint value is below the upper joint limit
               if (ik_solution(i) < chain_bounds[i].max_position_){}

			   else
				   valid = false;


			 }

			 else
				 valid = false;

		 }

		 //Lower bound is smaller than upper bound
         if (chain_bounds[i].max_position_ < chain_bounds[i].min_position_)
		 {
			 //check if joint value is above the lower joint limit
             if (ik_solution(i) > chain_bounds[i].max_position_)
			 {
				//check if joint value is below the upper joint limit
               if (ik_solution(i) < chain_bounds[i].min_position_){}

			   else
				   valid = false;


			 }

			 else
				 valid = false;
		 }


	 }

	 return valid;

}


double ArticulatedObjectConstraints::compute_config_distance(KDL::JntArray ik_r_arm_config, std::vector<double> r_arm_config_reference)
{
	//Sum of joint distances to reference value
	double sum_distances = 0.0;

	//Compute the distances current joint value to reference joint value
	for(unsigned int i=0 ; i<r_arm_config_reference.size() ; i++)
	{
		sum_distances = sum_distances + fabs(r_arm_config_reference[i] - ik_r_arm_config(i));
	}

	return sum_distances;
}



}//END of namespace
