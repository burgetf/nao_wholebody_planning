/*
 * local_planner.h
 *
 *  Created on: May 11, 2012
 *      Author: Felix Burget
 */

#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <urdf_model/model.h>

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

#include<random_numbers/random_numbers.h>
#include <vector>

#ifndef LOCAL_PLANNER_H_
#define LOCAL_PLANNER_H_

namespace nao_planner {

class LocalPlanner
{
	public:
	LocalPlanner();
	~LocalPlanner();

	//Get the Pose of the Hip frame w.r.t the right foot frame
	bool getHipPose(std::vector<double> right_leg_config, KDL::Frame &hip_pose);

	//Check the joint limits
    //bool checkJointLimits(std::vector<double> config, std::vector< std::pair< double, double > > chain_bounds);
    bool checkJointLimits(std::vector<double> config, std::vector<moveit::core::VariableBounds> chain_bounds);

	//Check pose of left leg (i.e check if NAO is in double support)
	bool checkLeftLegPose(std::vector<double> legs_config);

	//Compute the IK for the left leg
    //bool computeLeftLegIK(std::vector<double> legs_config,std::vector<double>& new_l_leg_config, std::vector< std::pair< double, double > > bounds_left_leg);
    bool computeLeftLegIK(std::vector<double> legs_config,std::vector<double>& new_l_leg_config, std::vector<moveit::core::VariableBounds> bounds_left_leg);


	private:
	KDL::Chain chain_right_leg_;
	KDL::Chain chain_left_leg_;
	KDL::Chain chain_legs_;

	random_numbers::RandomNumberGenerator    rng_;

	KDL::Frame getDesiredLeftLegPose(KDL::Frame hip_transform);


};




}


#endif
