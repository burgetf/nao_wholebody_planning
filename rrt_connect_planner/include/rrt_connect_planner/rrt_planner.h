/*
 * RRT_Planner.h
 *
 *  Created on: May 10, 2012
 *      Author: Felix Burget
 */

#ifndef RRT_PLANNER_H_
#define RRT_PLANNER_H_


#include <hrl_kinematics/TestStability.h>
#include <rrt_connect_planner/local_planner.h>
#include <rrt_connect_planner/articulated_object_constraints.h>

#include <moveit_msgs/DisplayTrajectory.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>



namespace nao_planner {

//Structure representing a node of a tree (i.e a configuration)
struct Node{
	int index;
	int predecessor_index;
	std::vector<double> config;
	int cart_hand_pose_index;	//Only used for motion planning with articulated object constraints
};

//Structure representing a tree (set of connected configurations)
struct Tree{

	std::string name;
	std::vector<Node> nodes;
	int num_nodes;
};


enum Status { ADVANCED, REACHED, TRAPPED };


class RRT_Planner
{
public:
    //RRT_Planner(boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm, const std::string &group_name);
    RRT_Planner(boost::shared_ptr<planning_scene::PlanningScene> p_s, const std::string &group_name);
	virtual ~RRT_Planner();

	//Load double support configuration database
    void load_DS_database(char* stat_stable_config_file);

	//Set Start and Goal configuration
	bool setStartGoalConfigs(std::vector<double> start_config , std::vector<double> goal_config);

	//Set the joint weights (allows to disable joints for planning/ to move specific joints with preference)
	void setJointWeights(std::vector<double> jw);

	//Function to scale the support polygon (used for stat.-stability check)
	void scale_support_polygon(float scale_sp);

	//Solve the Query
    moveit_msgs::DisplayTrajectory solveQuery(int max_iter, double max_step_size, char *solution_file_path, int &num_nodes_generated, float &time_elapsed);

	//Check further constraints -> considered in the isStateValid function
    //bool isFeasible(const robot_state::RobotState& state, bool verbose);
    bool isFeasible(robot_state::RobotState state, bool verbose);

	//Activate a constraint for manipulating an articulated object
	void activate_drawer_constraint(std::vector<double> cart_pose_hand_start,std::vector<double> cart_pose_hand_goal, int num_interp_points, double translation_length);

	//Activate a constraint for manipulating an articulated object
	void activate_door_constraint(std::vector<double> cart_pose_hand_start,std::vector<double> cart_pose_hand_goal, int num_interp_points, std::vector<double> door_param);

	//Set the Trajectory Mode (to simulation or to execution)
	void setTrajectoryMode(bool execute_trajectory);


protected:
	ros::NodeHandle root_nh_;
	ros::NodeHandle nh_;

    //Planning Scene Monitor
    //boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm_;

    //Planning Scene
    boost::shared_ptr<planning_scene::PlanningScene> p_s_;

	//Name of Planning Group
	std::string group_name_;


	//Stability constraint
	hrl_kinematics::TestStability stability_;

	//Constraint Kinematics for NAO
	nao_planner::LocalPlanner local_planner_;

	//Articulated Object constraint
	nao_planner::ArticulatedObjectConstraints object_manipulation_;
	//Flag indicating whether articulation_constraints is active/inactive
	bool articulation_constraint_active_;
	//Velocity applied for manipulating objects
	float manipulation_velocity_;
	//Timesteps for manipulation configurations
	float time_steps_manipulation_;


	//Joint limits
    //std::vector< std::pair< double, double > > joint_limits_;
    std::vector<moveit::core::VariableBounds> joint_limits_;


	//Number of joint of plannning group "group_name_"
	int num_joints_;


	//Array containing the statically stable double support configurations
	double **ss_configs_;
	//Number of statically stable configurations in the file
	int num_configs_;

	//Vector containing the joint weight (used to set a preference for joints to move)
	std::vector<double> joint_weights_;


	//From constraint_sampler.h
	planning_scene::PlanningSceneConstPtr                   scene_;

	//RRT Trees (one growing from the start and another one from the goal configuration)
	Tree T_start_;
	Tree T_goal_;


	//Random number generator for generating random configs
	random_numbers::RandomNumberGenerator                   rng_;


	//Trajectory execution mode (only simulate trajectory -> traj_mode_= false / or execute trajectory -> traj_mode_= true)
	bool execute_trajectory_;

private:


    //Joint Names Array and function to bring the order of the joints in the RobotModel in the order used for planning
    std::vector<std::string> joint_names_;
    std::vector<std::string> joint_name_order_RobModel_to_Planner(const std::vector<std::string> j_names);


	//Get a random sample from the array "ss_configs_"
	void getRandomStableConfig(Node &sample);

	//Searches for the nearest neighbor of q_rand according to some distance metric
	void findNearestNeighbour(Tree T_current, Node q_rand, Node& nearest_neighbor);


	//Generate a q_new along the line connecting q_near to q_rand
	Status generate_q_new (Node q_near,Node q_rand, Node &q_new, double distance);

	//Modify q_new such that the returned node obeys the constraints (Joint limits + double support)
	Status enforce_DS_Constraints(Node q_new, Node &q_new_modified);

	//Modify q_new_modified such that the returned node obeys the manipulation constraints (hand moves along line or circle)
	Status enforce_MA_Constraints(std::string tree_name, Node q_near,Node q_new_ds, Node &q_new_manip);

	//Check whether q_new_modified is valid (No Collisions + Stat.-Stable)
	bool isConfigValid(Node q_new_modified);

	//extend the tree (finds a valid q_new and adds it to the tree)
	Status extendTree(Tree& tree, Node q_rand, Node& q_near, double max_step_size);

	//connect the trees (tries to connect one tree to the other)
	Status connectTree(Tree& tree, Node q_rand, Node& q_near, double max_step_size);

	//Add a configuration to the tree
	void addConfigtoTree(Tree& tree, Node q_near ,Node q_new_modified);

	//Write the solution path configurations into a file
    void writePath(Tree T_start, Tree T_goal, Node q_near, int connector, moveit_msgs::DisplayTrajectory &trajectory, char *solution_file_path);

	//Interpolate two waypoints of the raw solution path
	moveit_msgs::DisplayTrajectory interpolateWaypoints(std::vector<double> waypoint_start,std::vector<double> waypoint_goal, int num_intermediate_waypoints);

	//Interpolate the waypoints of the shortcutted path
	void interpolateShortPathWaypoints(std::vector<std::vector<double> > short_path, std::vector<std::vector<double> > &interpolated_path, int num_intermediate_waypoints);

	//Path Shortcutter
	//Given the raw solution path tries to reduce the number of waypoints/configurations while maintaining validity of the path
	bool path_shortcutter(std::vector< std::vector<double> > raw_solution_path, std::vector< std::vector<double> > &shortcutted_path, int num_intermediate_waypoints);

	//Generate a Trajectory from the array "path"
	void generateTrajectory(std::vector< std::vector<double> > path, int num_configs, moveit_msgs::DisplayTrajectory &traj);



};

} /* namespace nao_rrt_planner */
#endif /* RRT_PLANNER_H_ */ 
