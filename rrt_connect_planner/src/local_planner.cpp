/*
 * local_planner.cpp
 *
 *  Created on: May 11, 2012
 *      Author: Felix Burget
 */

#include "rrt_connect_planner/local_planner.h"
#include <math.h>
//#include<sys/time.h>


namespace nao_planner {

LocalPlanner::LocalPlanner()
{
	//Definition of a Kinematic chain for right leg
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.0,0.04519))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1029))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_right_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));

	//Definition of a Kinematic chain for left leg
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,-0.1))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,-0.1029))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_left_leg_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,-0.04519))));

	//Definition of a Kinematic chain for legs (r_foot to hip to l_foot)
	// From right foot to hip
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.0,0.04519))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1029))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.1))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));
    // From hip to left foot
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None),KDL::Frame(KDL::Vector(0.0,0.05,0.0))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,-0.1))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,-0.1029))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),KDL::Frame(KDL::Vector(0.0,0.0,0.0))));
	chain_legs_.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,-0.04519))));

}


LocalPlanner::~LocalPlanner()
{
	// TODO Auto-generated destructor stub
}



KDL::Frame LocalPlanner::getDesiredLeftLegPose(KDL::Frame hip_transform)
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
	//std::cout << "Desired pose for left foot (w.r.t hip frame))"<< std::endl << desiredPoseLeftLeg <<std::endl;

	return desiredPoseLeftLeg;

}




//Compute the pose of the hip given the configuration of the right foot (r_foot fixed on the floor)
bool LocalPlanner::getHipPose(std::vector<double> right_leg_config, KDL::Frame &hip_pose)
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




//bool LocalPlanner::computeLeftLegIK(std::vector<double> legs_config,std::vector<double>& new_l_leg_config,std::vector< std::pair< double, double > >  bounds_left_leg)
bool LocalPlanner::computeLeftLegIK(std::vector<double> legs_config,std::vector<double>& new_l_leg_config, std::vector<moveit::core::VariableBounds>  bounds_left_leg)
{
	//Get the right leg configuration
	std::vector<double> right_leg_config(5);
	for (std::size_t i = 0 ; i < 5 ; i++)
	{
	 right_leg_config[i] = legs_config[i];
	}

	//Get the left leg configuration
	std::vector<double> left_leg_config(5);
	for (std::size_t i = 5 ; i < 10 ; i++)
	{
	 left_leg_config[i-5] = legs_config[i];
	}

	//Compute the hip pose (w.r.t the right foot frame)
	KDL::Frame hip_pose;
	getHipPose(right_leg_config,hip_pose);


	//Get the desired pose for the left Leg (w.r.t the hip frame)
	KDL::Frame desiredLeftLegPose;
	desiredLeftLegPose = getDesiredLeftLegPose(hip_pose);



	//+++++++++++++++ Solve the IK for the left leg
	//Configure kinematic solvers
	 KDL::ChainFkSolverPos_recursive fksolver1 = KDL::ChainFkSolverPos_recursive(chain_left_leg_);
	 KDL::ChainIkSolverVel_pinv vik = KDL::ChainIkSolverVel_pinv(chain_left_leg_);
	 KDL::ChainIkSolverPos_NR ik= KDL::ChainIkSolverPos_NR(chain_left_leg_,fksolver1,vik,100,1e-3);

	 //Number of joints constituting the chain
	 unsigned int num_joints = chain_left_leg_.getNrOfJoints();


	 //Joint array to store result
	 KDL::JntArray q_out= KDL::JntArray(num_joints);


	  //Take the current state of the left leg as the seed state for the IK algorithm
	 KDL::JntArray q_init = KDL::JntArray(num_joints);
	  for (unsigned int i = 0 ; i < 5 ; ++i)
	  {
		q_init(i) = left_leg_config[i];
	  }

	  //timeval start, end;
	  //gettimeofday(&start,0);
	  //Compute a IK solution (no joint limits)
	  int ik_check;
	  ik_check = ik.CartToJnt(q_init,desiredLeftLegPose,q_out);
	  //gettimeofday(&end,0);


	  //If an IK solution has been found check if it respects the joint limits
	  bool valid_solution = false;
	  if (ik_check>=0)
	  {
		  std::vector<double> ik_solution(5);
		  for (unsigned int i = 0 ; i < 5 ; i++)
		  {
			  ik_solution[i] = q_out(i);
		  }
		  valid_solution = checkJointLimits(ik_solution,bounds_left_leg);

		  if (valid_solution == false)
		  {
			  //std::cout<<"IK solution is invalid (joint limits violated)" <<std::endl;
		  }
		  else
		  {
			  //Assign the IK solution to the new_l_leg_config vector
			  for (unsigned int i = 0 ; i < 5 ; i++)
			  {
				  new_l_leg_config[i] = ik_solution[i];
			  }

		  }

	  }

	  //Return valid_solution flag
	  return valid_solution;
}






//Check whether the left foot has the correct pose w.r.t the right foot
bool LocalPlanner::checkLeftLegPose(std::vector<double> legs_config)
{
	// ------------------------------------------------FK for legs chain ---------------------------------------------
	//Pose of left foot
	KDL::Frame l_foot;

	// Create solver based on kinematic chain
	KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain_legs_);

	// Create joint array
	unsigned int nj = chain_legs_.getNrOfJoints();
	KDL::JntArray jointpositions = KDL::JntArray(nj);


	// Assign values to the joint positions array
	for(unsigned int i=0;i<nj;i++){
		jointpositions(i)= legs_config[i];
	}

	// Calculate forward position kinematics (pose of hip)
	bool kinematics_status;
	kinematics_status = fksolver.JntToCart(jointpositions,l_foot);
	 //std::cout << "FK solution for right leg:  "<< std::endl;
	if(kinematics_status>=0)
	{
		//std::cout << "Pose of left leg w.r.t right leg"<< std::endl << l_foot <<std::endl;
	}
	else{
		printf("%s \n","ERROR: failed to calculate forward kinematics for legs:(");
	}


	//---------  Check if left foot has correct relative position w.r.t right foot ------------------------
	//Desired pose of l_foot w.r.t right foot
	KDL::Rotation des_rotation(1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,1.0);
	KDL::Vector des_position(0.0 , 0.1 , 0.0);
	KDL::Frame l_foot_desired(des_rotation,des_position);

	//Compute error between desired and current left foot location (only checking z/ height of foot for now)
	std::vector <double> pos_error(3);
	double max_pos_error = 0.0;
	for (int i = 0 ; i< 3 ; i++)
	{
		pos_error[i] = l_foot_desired.operator()(i,3) - l_foot.operator()(i,3);
		if (pos_error[i] > max_pos_error)
			max_pos_error = pos_error[i];
	}

	//Check the maximum position error (in x y z)
	if (max_pos_error > 0.01)
		return false;

	else
		return true;

}



//Check if a certain configuration of the left leg respects the joint limits
//bool LocalPlanner::checkJointLimits(std::vector<double> config, std::vector< std::pair< double, double > > chain_bounds)
bool LocalPlanner::checkJointLimits(std::vector<double> config, std::vector<moveit::core::VariableBounds> chain_bounds)
{
	bool valid = true;
	 for(unsigned int i=0 ; i<chain_bounds.size() ; i++)
	 {
//		 std::cout<<chain_bounds[i].min_position_<<std::endl;
//		 std::cout<<chain_bounds[i].max_position_<<std::endl;
//		 std::cout<<std::endl;

		 //Lower bound is smaller than upper bound
         if (chain_bounds[i].min_position_ < chain_bounds[i].max_position_)
		 {
			 //check if joint value is above the lower joint limit
             if (config[i] > chain_bounds[i].min_position_)
			 {
				//check if joint value is below the upper joint limit
               if (config[i] < chain_bounds[i].max_position_){}

			   else
			   {
				   //std::cout <<" Joint value " << config[i] <<" is bigger than " << chain_bounds[i].second <<std::endl;
				   //std::cout << i << std::endl;
				   valid = false;
			   }

			 }

			 else
			 {
                 //std::cout <<" Joint value " << config[i] <<" is smaller than " << chain_bounds[i].min_position_<<std::endl;
				 //std::cout << i << std::endl;
				 valid = false;
			 }
		 }

		 //Lower bound is smaller than upper bound
         if (chain_bounds[i].max_position_ < chain_bounds[i].min_position_)
		 {
			 //check if joint value is above the lower joint limit
             if (config[i] > chain_bounds[i].max_position_)
			 {
				//check if joint value is below the upper joint limit
               if (config[i] < chain_bounds[i].min_position_){}

			   else
			   {
                   std::cout <<" Joint value " << config[i] <<" is bigger than " << chain_bounds[i].max_position_ <<std::endl;
				   std::cout << i << std::endl;
				   valid = false;
			   }

			 }

			 else
			 {
                 //std::cout <<" Joint value " << config[i] <<" is smaller than " << chain_bounds[i].min_position_<<std::endl;
				 //std::cout << i << std::endl;
				 valid = false;
			 }
		 }


	 }

	 return valid;

}



}
