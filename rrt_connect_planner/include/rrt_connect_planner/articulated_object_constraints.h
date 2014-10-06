 /*
 * articulated_object_constraints.h
 *
 *  Created on: Jul 17, 2012
 *      Author: Felix Burget
 */

#ifndef ARTICULATED_OBJECT_CONSTRAINTS_H_
#define ARTICULATED_OBJECT_CONSTRAINTS_H_

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

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>

namespace nao_planner {


class ArticulatedObjectConstraints
{
	public:
	ArticulatedObjectConstraints();
	~ArticulatedObjectConstraints();

	//Add drawer constraint
	void add_drawer_constraint(std::vector<double> pose_hand_start,std::vector<double> pose_hand_goal, int num_interp_points);

	//Add door constraint
	void add_door_constraint(std::vector<double> pose_hand_start,std::vector<double> pose_hand_goal, int num_interp_points, std::vector<double> door_param);

	//Get the number of cartesian points on the hand trajectory
	int get_num_trajectory_points();

	//Check if the right arm is at a certain cartesian pose
	bool right_hand_at_pose(int des_cart_hand_pose_index, std::vector<double> r_leg_config, std::vector<double> r_arm_config);

	//Compute IK for right arm given a cartesian pose
    //bool compute_right_arm_IK(int des_hand_pose_index, std::vector<double> r_leg_config, std::vector<double> r_arm_config_reference, std::vector< std::pair< double, double > >  bounds_right_arm, std::vector<double>& new_r_arm_config);
    bool compute_right_arm_IK(int des_hand_pose_index, std::vector<double> r_leg_config, std::vector<double> r_arm_config_reference, std::vector<moveit::core::VariableBounds> bounds_right_arm, std::vector<double>& new_r_arm_config);



	protected:



	private:
	KDL::Chain chain_right_leg_;
	KDL::Chain chain_r_leg_arm_;

	//Vector representing the hand position and direction of the right hand frame's z-axis w.r.t right foot frame
	std::vector<double> pose_hand_start_tree_;	//for start hand pose (x,y,z,Xz,Yz,Zz)
	std::vector<double> pose_hand_goal_tree_;   //for goal hand pose

	//Stores the number of intermediate points on the object trajectory
	int num_intermediate_points_;

	//Trajectory points (cartesian)
	double **hand_trajectory_points_;

	//Compute num_intermediate_points_ points on the trajectory
	void computeDrawerTrajectoryPoints();

	void computeDoorTrajectoryPoints(std::vector<double> door_param);

	//Compute the Pose of the Hip
	bool getHipPose(std::vector<double> right_leg_config, KDL::Frame &hip_pose);

	//Check if a right arm configuration respects the joint limits
    //bool check_joint_limits(KDL::JntArray ik_solution, std::vector< std::pair< double, double > > chain_bounds);
    bool check_joint_limits(KDL::JntArray ik_solution, std::vector<moveit::core::VariableBounds> chain_bounds);

	//Compute the distance between two right arm configurations
	double compute_config_distance(KDL::JntArray ik_r_arm_config, std::vector<double> r_arm_config_reference);


}; //END of class




} //END of namespace


#endif //END of define
