/*
 * nao_constraint_kinematics.cpp
 *
 *  Created on: May 11, 2012
 *      Author: Felix Burget
 */

#include "rrt_connect_planner/nao_constraint_kinematics.h"
#include <math.h>
//#include<sys/time.h>

#include <kdl/utilities/svd_eigen_HH.hpp>
#include <Eigen/Eigenvalues>


namespace nao_constraint_sampler {

#include "ikfast_TranslationDirection5D.cpp"

ConstraintKinematics::ConstraintKinematics()
{
	//Definition of a kinematic chain for right leg
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.0,0.04519))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1029))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));

	//Definition of a kinematic chain for right leg
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,-0.1))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,-0.1029))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,-0.04519))));

	//Definition of a kinematic chain for right arm
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.0,0.182))));
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,-0.098,0.0))));
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.105,-0.015,0.0))));
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.05595,0.0,0.0))));
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.05775,0.0,-0.01231))));
	//Offset to grasp frame (required to make KDL and OpenRave endeffector frame identical)
	chain_right_arm_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Rotation::RPY(M_PI_2,0.0,0.0), KDL::Vector(0.01,0.0,0.0))));

	//Init the desired right hand position
	desired_right_hand_position_.resize(3);
	desired_right_hand_position_[0] = 0.0;
	desired_right_hand_position_[1] = 0.0;
	desired_right_hand_position_[2] = 0.0;

	//Init the desired right hand direction
	desired_right_hand_direction_.resize(3);
	desired_right_hand_direction_[0] = 0.0;
	desired_right_hand_direction_[1] = -1.0;
	desired_right_hand_direction_[2] = 0.0;

}

ConstraintKinematics::~ConstraintKinematics()
{
	// TODO Auto-generated destructor stub
}



//bool ConstraintKinematics::computeLeftLegIK(KDL::Frame hip_transform, std::vector< std::pair< double, double > > bounds_left_leg,std::vector<double>& left_leg_config,int max_ik_iter)
bool ConstraintKinematics::computeLeftLegIK(KDL::Frame hip_transform, std::vector<moveit::core::VariableBounds> bounds_left_leg,std::vector<double>& left_leg_config,int max_ik_iter)
{
	// ----------------------------------------- Compute required pose for left foot/endeffector (w.r.t hip frame) ----------------------------------------
	//Desired Rotation of left foot w.r.t hip
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

	//Compute the transpose rotation (hip to right leg frame -> same rot as hip to left leg frame)
	KDL::Rotation rotation (hip_pose_transpose[0],hip_pose_transpose[1],hip_pose_transpose[2],hip_pose_transpose[3],
			hip_pose_transpose[4],hip_pose_transpose[5],hip_pose_transpose[6],hip_pose_transpose[7],hip_pose_transpose[8]);



	//Desired Position of left foot w.r.t the right foot (fixed relative position)
	KDL::Vector pos_l_foot_fixed(0.0 , 0.1 , 0.0);
	//Get the position vector right foot to hip
	KDL::Vector pos_hip = hip_transform.p;


	std::vector<double> pos_l_foot_from_hip(3);
	double tmp= 0.0, tmp2 = 0.0;
	for (unsigned int i=0;i<3;i++)
			{
				 tmp = -rotation.operator()(i,0) * pos_hip.x() - rotation.operator()(i,1) * pos_hip.y() - rotation.operator()(i,2) * pos_hip.z();
				 tmp2 = rotation.operator()(i,0) * pos_l_foot_fixed.x() + rotation.operator()(i,1) * pos_l_foot_fixed.y() + rotation.operator()(i,2) * pos_l_foot_fixed.z();
				 pos_l_foot_from_hip[i] = tmp + tmp2;
			}


	//Position of landing point in hip frame
	KDL::Vector position(pos_l_foot_from_hip[0], pos_l_foot_from_hip[1], pos_l_foot_from_hip[2]);



	//Desired Pose for Left Foot (w.r.t hip frame)
	KDL::Frame desiredPoseLeftLeg(rotation,position);

	//Write the desiredPoseLeftLeg
	//std::cout << "Pose of hip (w.r.t r foot frame))"<< std::endl << hip_transform <<std::endl;
	//std::cout << "Desired pose for left foot (w.r.t hip frame))"<< std::endl << desiredPoseLeftLeg <<std::endl;


	// ------------------------------------------------------ IK left leg chain ---------------------------------------------------------------

	//Configure kinematic solvers
	 KDL::ChainFkSolverPos_recursive fksolver1 = KDL::ChainFkSolverPos_recursive(chain_left_leg_);
	 KDL::ChainIkSolverVel_pinv vik = KDL::ChainIkSolverVel_pinv(chain_left_leg_);
	 KDL::ChainIkSolverPos_NR ik= KDL::ChainIkSolverPos_NR(chain_left_leg_,fksolver1,vik,100,1e-3);

	 //Number of joints constituting the chain
	 unsigned int num_joints = chain_left_leg_.getNrOfJoints();


	 //Joint array to store result
	 KDL::JntArray q_out= KDL::JntArray(num_joints);


	 //Compute IK and check whether joint limits of left leg are respected
	 //bool ik_check;
	 int ik_check;
	 bool valid_solution = false;
	 //Initial guess
	 KDL::JntArray q_init = KDL::JntArray(num_joints);
	 //Compute IK solutions until a valid one is obtained or the max. number of IK iterations is reached
	 for(int iter = 0 ; iter < max_ik_iter ; iter++)
	 {
		  //Generate a seed state for the IK algorithm (random configuration within joint limits of left leg))
		  for (unsigned int i = 0 ; i < bounds_left_leg.size() ; ++i)
		  {
            //q_init(i) = rng_.uniformReal(bounds_left_leg[i].min_position_, bounds_left_leg[i].max_position_);
			q_init(i) =  -r_leg_config_[i];
			//std::cout <<" q_init : "<< q_init(i) << std::endl;
		  }

		  //timeval start, end;
		  //gettimeofday(&start,0);
		  //Compute a IK solution (no joint limits)
		  ik_check = ik.CartToJnt(q_init,desiredPoseLeftLeg,q_out);
		  //gettimeofday(&end,0);


		  if (ik_check>=0)
		  {
			  //Check if IK solution respects the joint limits
			  valid_solution = joint_limits_respected(q_out,bounds_left_leg);
			  if (valid_solution == true)
			  {
				  break;
			  }
			  else
			  {
				  //std::cout<<" Current IK solution is invalid (joint limits violated)" <<std::endl;
			  }
		  }

	 }



	 //Print joint values if an IK solution respecting the joint limits is found
	 if(ik_check>=0 && valid_solution == true)
	 {
		 //std::cout << "IK solution for left leg:  "<< std::endl;
		 for(unsigned int i=0;i<num_joints;i++)
		 {
			 //Assign IK solution to referenced left_leg_config array
			 left_leg_config[i] = q_out(i);
			 //std::cout << q_out(i) << std::endl;
		 }
		 return true;
	 }
	 else
	 {
	 		//printf("%s \n","ERROR: No valid IK solution found :(");
	 		return false;
	 }

}



void ConstraintKinematics::desiredRightArmPose(std::vector<double> right_hand_pose)
{
	//Right hand position w.r.t right foot frame
	desired_right_hand_position_[0] = right_hand_pose[0];
	//std::cout << "X : "<< desired_right_hand_position_[0]<< std::endl;
	desired_right_hand_position_[1] = right_hand_pose[1];
	//std::cout << "Y : "<< desired_right_hand_position_[1]<< std::endl;
	desired_right_hand_position_[2] = right_hand_pose[2];
	//std::cout << "Z : "<< desired_right_hand_position_[2]<< std::endl;

	//Right hand direction w.r.t right foot frame
	desired_right_hand_direction_[0] = right_hand_pose[3];
	//std::cout << "x component of z-axis : "<< desired_right_hand_direction_[0]<< std::endl;
	desired_right_hand_direction_[1] = right_hand_pose[4];
	//std::cout << "y component of z-axis : "<< desired_right_hand_direction_[1]<< std::endl;
	desired_right_hand_direction_[2] = right_hand_pose[5];
	//std::cout << "z component of z-axis : "<< desired_right_hand_direction_[2]<< std::endl;

}



//bool ConstraintKinematics::computeRightArmIK(KDL::Frame hip_transform, std::vector< std::pair< double, double > > bounds_right_arm ,std::vector<double>& right_arm_config, double& ergonomics_index)
bool ConstraintKinematics::computeRightArmIK(KDL::Frame hip_transform, std::vector<moveit::core::VariableBounds> bounds_right_arm ,std::vector<double>& right_arm_config, double& ergonomics_index)
{

	// -------------------------------------- Compute Position of right Hand in Hip and Torso frame -----------------------------
	//Desired Position of right hand w.r.t the right foot
	//		std::cout << "X : "<< desired_right_hand_position_[0]<< std::endl;
	//		std::cout << "Y : "<< desired_right_hand_position_[1]<< std::endl;
	//		std::cout << "Z : "<< desired_right_hand_position_[2]<< std::endl;
	KDL::Vector pos_r_hand_desired(desired_right_hand_position_[0], desired_right_hand_position_[1] , desired_right_hand_position_[2]);
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


	// -------------------------------------- Compute Direction of right Hand in Torso frame -----------------------------
	//Compute desired direction of right hand in torso frame
	std::vector<double> direction_r_hand_from_torso(3);
	for (unsigned int i = 0 ; i < 3 ; i++)
			{
			 direction_r_hand_from_torso[i] = rotation.operator()(i,0) * desired_right_hand_direction_[0] + rotation.operator()(i,1) * desired_right_hand_direction_[1] + rotation.operator()(i,2) * desired_right_hand_direction_[2];
			}


	//Solve only position IK (orientation can be arbitrary)
	//TODO The KDL Solver below can only solve the IK when position and orientation has been specified!!!!
	// FIND A WAY TO SOLVE ONLY POSITION IK
	if (desired_right_hand_direction_[0] == 0.0 && desired_right_hand_direction_[1] == 0.0 && desired_right_hand_direction_[2] == 0.0 )
	{

		// ------------------------------------ Solve IK with KDL (only right hand position considered) -------------------------------------------------
		//Desired Pose for Left Foot (w.r.t hip frame)
		KDL::Vector pos_r_hand_in_hip_frame = KDL::Vector::Zero();
		//Set values of vector
		pos_r_hand_in_hip_frame.x(pos_r_hand_from_hip[0]);
		pos_r_hand_in_hip_frame.y(pos_r_hand_from_hip[1]);
		pos_r_hand_in_hip_frame.z(pos_r_hand_from_hip[2]);

		//Create goal frame
		KDL::Frame desiredRightHandPose(pos_r_hand_in_hip_frame);

		//Configure kinematic solvers
		 KDL::ChainFkSolverPos_recursive fksolver1 = KDL::ChainFkSolverPos_recursive(chain_right_arm_);
		 KDL::ChainIkSolverVel_pinv vik = KDL::ChainIkSolverVel_pinv(chain_right_arm_);
		 KDL::ChainIkSolverPos_NR ik= KDL::ChainIkSolverPos_NR(chain_right_arm_,fksolver1,vik,100,1e-3);

		 //Number of joints constituting the chain
		 unsigned int num_joints = chain_right_arm_.getNrOfJoints();


		 //Joint array to store result
		 KDL::JntArray q_out= KDL::JntArray(num_joints);


		 //Compute IK and check whether joint limits of left leg are respected
		 //bool ik_check;
		 int ik_check;
		 bool valid_solution = false;
		 //Initial guess
		 KDL::JntArray q_init = KDL::JntArray(num_joints);
		 //Maximum tries to solve IK
		 int max_ik_iter = 5;
		 //Compute IK solutions until a valid one is obtained or the max. number of IK iterations is reached
		 for(int iter = 0 ; iter < max_ik_iter ; iter++)
		 {
			  //Generate a seed state for the IK algorithm (random configuration within joint limits of right arm))
			  for (unsigned int i = 0 ; i < bounds_right_arm.size() ; ++i)
			  {
                q_init(i) = rng_.uniformReal(bounds_right_arm[i].min_position_, bounds_right_arm[i].max_position_);
				//std::cout <<" q_init : "<< q_init(i) << std::endl;
			  }

			  //Compute a IK solution (no joint limits)
			  ik_check = ik.CartToJnt(q_init,desiredRightHandPose,q_out);


			  if (ik_check>=0)
			  {
				  //Check if IK solution respects the joint limits
				  valid_solution = joint_limits_respected(q_out,bounds_right_arm);
				  if (valid_solution == true)
				  {
					  break;
				  }
				  else
				  {
					  //std::cout<<" Current IK solution is invalid (joint limits violated)" <<std::endl;
				  }
			  }
		 }

		 //Print joint values if an IK solution respecting the joint limits is found
		 if(ik_check>=0 && valid_solution == true)
		 {
			 //std::cout << "IK solution for right arm:  "<< std::endl;
			 for(unsigned int i = 0 ; i < num_joints ; i++)
			 {
				 //Assign IK solution to referenced right_arm_config array
				 right_arm_config[i] = q_out(i);
				 //std::cout << q_out(i) << std::endl;
			 }
			 return true;
		 }
		 else
		 {
				//printf("%s \n","ERROR: No valid IK solution found :(");
				return false;
		 }

	}


	else
	{
		// ------------------------------------------- Solve IK with openRave -------------------------------------------------
		//position
		IKReal position[3];
		position[0] = pos_r_hand_from_torso[0];
		position[1] = pos_r_hand_from_torso[1];
		position[2] = pos_r_hand_from_torso[2];
		IKReal *pos = position;
		//direction
		IKReal direction[9]; //First three values define target direction of EE z-axis
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
		IKSolver ik_solver;
		//Solve the IK
		if(ik_solver.ik(pos,dir,pfree,solutions))
		{
			//Number of solutions found
			//std::cout<< "Number of solutions found: " << solutions.size()<<std::endl;

			//Number of Joints
			int num_joints = getNumJoints();

			//Init Loop parameters
			std::vector<IKReal> sol(num_joints);
			bool valid_solution = false;
			KDL::JntArray joint_values = KDL::JntArray(num_joints);

			//Ergonomics of the best arm posture found so far
			//double best_ergonomic_index = 10000.0;
			double best_ergonomic_index = 0.0;

			//Ergonomics of the current arm posture
			double current_ergonomic_index = 0.0;
			//Index of the best arm posture found so far
			int best_arm_posture_index = 1000;

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
				 valid_solution = joint_limits_respected(joint_values,bounds_right_arm);


				//If solution respects joint limits
				 if (valid_solution == true)
				 {

					 //Compute the ergonomics of the current arm posture (sum of the distances to reference posture)
					 current_ergonomic_index = compute_ergonomics_index(joint_values,rotation);

					 //If the ergonomics of the current arm posture is better than the best one found so far.....
					//if (current_ergonomic_index < best_ergonomic_index)
					//{
					//	best_ergonomic_index = current_ergonomic_index;
					//	best_arm_posture_index = solution;
					//}
					 if (current_ergonomic_index > best_ergonomic_index)
					 {
						 best_ergonomic_index = current_ergonomic_index;
						 best_arm_posture_index = solution;
					 }
				 }

			}//End of IK Solution loop


			//When a valid IK solution has been found
			if (best_arm_posture_index != 1000)
			{
				//Set the ergonomy index returned by reference
				ergonomics_index = best_ergonomic_index;

				//Get the best IK solution found
				std::vector<IKReal> vsolfree(solutions[best_arm_posture_index].GetFree().size());
				solutions[best_arm_posture_index].GetSolution(&sol[0],vsolfree.size() > 0 ? &vsolfree[best_arm_posture_index]:NULL);

				for(unsigned int i = 0 ; i < sol.size() ; i++)
				{
					//Assign IK solution to referenced left_leg_config array
					right_arm_config[i] = sol[i];
					//std::cout << right_arm_config[i] << std::endl;
				}

				//std::cout << "IK done" << std::endl;
				return true;
			}
			else
			{
				//No valid solution has been found
				return false;
			}

		}//End "Solve IK"

		//No IK solution has been found
		else
			return false;
	}


}




//Compute the pose of the hip given the configuration of the right foot (r_foot fixed on the floor)
bool ConstraintKinematics::getHipPose(std::vector<double> right_leg_config, KDL::Frame &hip_pose)
{
	// ------------------------------------------------FK right leg chain ---------------------------------------------
	// Create solver based on kinematic chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain_right_leg_);

	//Store right leg config for left leg IK computation
	r_leg_config_ = right_leg_config;

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
//bool ConstraintKinematics::joint_limits_respected(KDL::JntArray ik_solution, std::vector< std::pair< double, double > > chain_bounds)
bool ConstraintKinematics::joint_limits_respected(KDL::JntArray ik_solution, std::vector<moveit::core::VariableBounds> chain_bounds)
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
			   {
				   //std::cout << "Joint value: "<<ik_solution(i)<<" is above upper bound: "<<chain_bounds[i].second<<std::endl;
				   valid = false;
			   }

			 }

			 else
			 {
				 //std::cout << "Joint value: "<<ik_solution(i)<<" is below lower bound: "<<chain_bounds[i].first<<std::endl;
				 valid = false;
			 }

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
			   {
				   //std::cout << "Joint value: "<<ik_solution(i)<<" is above upper bound: "<<chain_bounds[i].first<<std::endl;
				   valid = false;
			   }

			 }

			 else
			 {
				 //std::cout << "Joint value: "<<ik_solution(i)<<" is below lower bound: "<<chain_bounds[i].second<<std::endl;
				 valid = false;
			 }
		 }


	 }

	 return valid;

}


//Computes the sum of the distances to a reference posture
double ConstraintKinematics::compute_ergonomics_index(KDL::JntArray joint_values, KDL::Rotation hip_rot_transpose)
{
	//Get Jacobian for current configuration
	KDL::Jacobian r_arm_jac(joint_values.rows());
	KDL::ChainJntToJacSolver jac_solver(chain_right_arm_);
	jac_solver.JntToJac(joint_values,r_arm_jac);
	//std::cout<<r_arm_jac.rows()<<"  "<<r_arm_jac.columns()<<std::endl;


	//--------------- Compute determinant of jacobian (corresponds to manipulability ellipsoid volume)
	int jac_rows = r_arm_jac.rows();
	int jac_cols = r_arm_jac.columns();
	//Compute determinant
	double **tmp_matrix;
	int new_cols_number = jac_cols*2;
	tmp_matrix = new double*[jac_rows]; //creates a new array of pointers to double objects
	for(int i=0; i<jac_rows; i++)
	tmp_matrix[i]=new double[new_cols_number];

	//Copy Jacobian matrix (result [J J])
	for(int i = 0 ; i < jac_rows; i++)
	{
		for(int j = 0 ; j < jac_cols; j++)
		{
		  tmp_matrix[i][j] = r_arm_jac.operator ()(i,j);
		  tmp_matrix[i][j+jac_cols] = tmp_matrix[i][j];
		  //std::cout<<r_arm_jac.operator ()(i,j)<<"  ";
		}
		//std::cout<<std::endl;
	}
	//std::cout<<std::endl;

	//Forward and backward sum
	double sum_fw = 0.0;
	double sum_bw = 0.0;
	for(int i = 0 ; i < jac_cols; i++)
	{
		sum_fw = sum_fw + (tmp_matrix[0][i] * tmp_matrix[1][i+1] * tmp_matrix[2][i+2] * tmp_matrix[3][i+3] * tmp_matrix[4][i+4] * tmp_matrix[5][i+5]);
		sum_bw = sum_bw - (tmp_matrix[0][new_cols_number-(1+i)] * tmp_matrix[1][new_cols_number-(1+i)] * tmp_matrix[2][new_cols_number-(1+i)] * tmp_matrix[3][new_cols_number-(1+i)] * tmp_matrix[4][new_cols_number-(1+i)] * tmp_matrix[5][new_cols_number-(1+i)]);
	}

	double det_jac = fabs(sum_fw - sum_bw);

	//Punish configuration for elbow up config (i.e low value for RShoulderPitch = joint_values(0))
	//On the other hand promote arm configs with elbow down (i.e high value for RShoulderPitch)
	double ergonomy_index = joint_values(0) * det_jac;

	return ergonomy_index;

	//std::cout<<"Determinant of Jacobian: "<<det_jac << std::endl;

	//-------------------------------------------------------------------------------------


	//------------- Experiment for using dimensions of principal axis of manipulability ellipsoid
	//Convert position part of jacobian to eigen Matrix
//	Eigen::MatrixXd r_arm_jac_pos(3,joint_values.rows());
//	for (int i = 0; i<3 ; i++)
//	{
//		for (unsigned int j = 0; j<joint_values.rows() ; j++)
//			{
//				r_arm_jac_pos(i,j) = r_arm_jac.operator ()(i,j);
//			}
//	}
//
//	//Compute the transpose position jacobian
//	Eigen::MatrixXd r_arm_jac_pos_trans(joint_values.rows(),3);
//	r_arm_jac_pos_trans = r_arm_jac_pos.transpose();
//
//	//Compute J*J_trans
//	Eigen::MatrixXd j_J_trans = r_arm_jac_pos * r_arm_jac_pos_trans;
//
//
//	//Compute direction of r_foot x-axis in hip frame
//	Eigen::Vector3d u_x(hip_rot_transpose.operator()(0,0),hip_rot_transpose.operator()(1,0),hip_rot_transpose.operator()(2,0));
//	//Compute direction of r_foot y-axis in hip frame
//	Eigen::Vector3d u_y(hip_rot_transpose.operator()(1,0),hip_rot_transpose.operator()(1,1),hip_rot_transpose.operator()(2,1));
//
//
//	//Compute transmission ratios in x and y direction (w.r.t r_foot frame not hip frame)
//	// i.e intersection between a line and the ellipsoid
//	double transm_ratio_x = 1 / (std::sqrt(u_x.transpose() * j_J_trans.inverse() * u_x));
//	double transm_ratio_y = 1 / (std::sqrt(u_y.transpose() * j_J_trans.inverse() * u_y));
//
//	//Manipulability index (transmission vector in x-y plane, i.e horizontal plane)
//	double vel_manipulability = std::sqrt(transm_ratio_x * transm_ratio_x + transm_ratio_y * transm_ratio_y);
//
//
//	return vel_manipulability;




//	//Compute eigenvalues and eigenvectors
//	Eigen::EigenSolver<MatrixXd> es;
//	es.compute(j_J_trans, true);
//
//	//Eigenvalues
//	std::vector< std::complex<double> > e_values(3);
//	e_values[0] = es.eigenvalues()(0,0);
//	e_values[1] = es.eigenvalues()(1,0);
//	e_values[2] = es.eigenvalues()(2,0);
//	std::cout << "The eigenvalues of A are: " << e_values[0] <<"  "<<e_values[1]<<"  "<<e_values[2] << std::endl;
//	std::cout << "The radii of the ellipsoid are: "<<std::sqrt(e_values[0])<<"  "<<std::sqrt(e_values[1])<<"  "<<std::sqrt(e_values[2]) << std::endl;
//
//	//Eigenvectors
//	std::cout << "The eigenvectors/direction of principal ellipsoid axis are:" << std::endl;
//	std::cout << es.eigenvectors().col(0) << std::endl;
//	std::cout << es.eigenvectors().col(1) << std::endl;
//	std::cout << es.eigenvectors().col(2) << std::endl;


	//--------------------------------------------------------------------------------

//	KDL::SVD_HH svd(r_arm_jac);
//	std::vector<KDL::JntArray> U(6,KDL::JntArray(joint_values.rows()));;
//	//KDL::JntArray S(joint_values.rows());
//	KDL::JntArray S(joint_values.rows());
//	std::vector<KDL::JntArray> V(joint_values.rows(),KDL::JntArray(joint_values.rows()));
//	//KDL::JntArray tmp(joint_values.rows());
//	int maxiter = 50;
//
//	//Do a singular value decomposition of "r_arm_jac" with maximum
//	//iterations "maxiter", put the results in "U", "S" and "V"
//	//r_arm_jac = U*S*Vt
//	int ret = svd.calculate(r_arm_jac,U,S,V,maxiter);
//	//int flag = KDL::svd_eigen_HH(A,U,S,V,x);

//	for(int k = 0 ; k < r_arm_jac.columns(); k++)
//		std::cout<<S(k)<< " ";
//
//	std::cout<<std::endl;
	//------------------------------------------------------------------------------------------



	//return det_jac;
	//return sum_distances;


}



} //END of Namespace
