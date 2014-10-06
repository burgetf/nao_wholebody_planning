/*
 * nao_constraint_kinematics.h
 *
 *  Created on: May 11, 2012
 *      Author: Felix Burget
 */

#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/joint.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainjnttojacsolver.hpp>

#include<random_numbers/random_numbers.h>

#include <moveit/robot_state/robot_state.h>

#ifndef NAO_CONSTRAINT_KINEMATICS_H_
#define NAO_CONSTRAINT_KINEMATICS_H_

namespace nao_constraint_sampler {

class ConstraintKinematics
{
	public:
	ConstraintKinematics();
	~ConstraintKinematics();

	//Compute the Pose of the Hip
	bool getHipPose(std::vector<double> right_leg_config, KDL::Frame &hip_pose);

	//Compute a valid IK solution for the left leg
    //bool computeLeftLegIK(KDL::Frame hip_transform, std::vector< std::pair< double, double > > bounds_left_leg ,std::vector<double>& left_leg_config,int max_ik_iter = 5);
    bool computeLeftLegIK(KDL::Frame hip_transform, std::vector<moveit::core::VariableBounds> bounds_left_leg ,std::vector<double>& left_leg_config,int max_ik_iter = 5);


	//Set the desired position of the right arm (the orientation is always horizontal/ parallel to the ground)
	void desiredRightArmPose(std::vector<double> right_hand_pose);

	//Compute a valid IK solution for the right arm
    //bool computeRightArmIK(KDL::Frame hip_transform, std::vector< std::pair< double, double > > bounds_right_arm ,std::vector<double>& right_arm_config, double& ergonomics_index);
    bool computeRightArmIK(KDL::Frame hip_transform, std::vector<moveit::core::VariableBounds> bounds_right_arm ,std::vector<double>& right_arm_config, double& ergonomics_index);




	private:
	KDL::Chain chain_right_leg_;
	KDL::Chain chain_left_leg_;
	KDL::Chain chain_right_arm_;

	//RandomNumberGenerator
	random_numbers::RandomNumberGenerator    rng_;

	//Desired position of the right hand w.r.t right foot
	std::vector<double> desired_right_hand_position_;
	//Desired direction of the right hand w.r.t right foot frame
	std::vector<double> desired_right_hand_direction_;

	//Reference configuration for right arm (used to select best IK solution)
	std::vector<double> r_arm_ref_config_;

	//Current Configuration of the right leg
	std::vector<double> r_leg_config_;

    //bool joint_limits_respected(KDL::JntArray ik_solution,std::vector< std::pair< double, double > > chain_bounds);
    bool joint_limits_respected(KDL::JntArray ik_solution,std::vector<moveit::core::VariableBounds> chain_bounds);


	double compute_ergonomics_index(KDL::JntArray joint_values, KDL::Rotation hip_rot_transpose);


};




}


#endif
