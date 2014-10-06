/*
 * RRT_Planner.cpp
 *
 *  Created on: May 10, 2012
 *      Author: Felix Burget
 */

#include "rrt_connect_planner/rrt_planner.h"
#include <fstream>

//Trajectory Smoother Classes
//#include <trajectory_processing/clamped_cubic_spline_smoother.h>
//#include <moveit/trajectory_processing/iterative_smoother.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
//#include <trajectory_processing/numerical_differentiation_spline_smoother.h>

#include <ros/ros.h>


namespace nao_planner { 

//Constructor
//RRT_Planner::RRT_Planner(boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm, const std::string &group_name): nh_("~"), psm_(psm),group_name_(group_name)
RRT_Planner::RRT_Planner(boost::shared_ptr<planning_scene::PlanningScene> p_s, const std::string &group_name): nh_("~"), p_s_(p_s),group_name_(group_name)
{
	//Planning Scene
    //p_s = psm_->getPlanningScene();

    //planning_scene_monitor::LockedPlanningSceneRO p_s_RO(psm_);


	//-------------------------------------- RECONFIGURE THE Kin.Model---------- ----------------------------
    //Get the urdf and srdf model
    const boost::shared_ptr< const urdf::ModelInterface > urdf_nao = p_s_->getRobotModel()->getURDF();
    //const boost::shared_ptr< const srdf::Model > srdf_nao = ->getSrdfModel();
    const boost::shared_ptr< const srdf::Model > srdf_nao = p_s_->getRobotModel()->getSRDF();

    //Configure the planning scene (set root joint)
//    if(!p_s_()->configure(urdf_nao, srdf_nao, "r_sole"))
//    {
//	  std::cout<< "FAILED to configure planning scene!!!" << std::endl;
//    }

    //Print the name of the root joint
    ROS_INFO("Root JOINT is %s ",p_s_->getRobotModel()->getRootJointName().c_str());
    ROS_INFO("Root LINK is %s ",p_s_->getRobotModel()->getRootLink()->getName().c_str());

    //Get Joint Names of planning group
    const std::vector<std::string> joint_names = p_s_->getRobotModel()->getJointModelGroup(group_name_)->getJointModelNames();

//    for (int i = 0; i< joint_names.size(); i++)
//    {
//        std::cout<<"Joint name: "<<joint_names[i]<<std::endl;
//    }

    //Fix the order of the joint names
    joint_names_ = joint_name_order_RobModel_to_Planner(joint_names);


    //Get the number of joints
    num_joints_ = joint_names_.size();


    joint_limits_.resize(num_joints_);
    //Get the limits for the joints in group "group_name_"
    for(std::size_t i = 0 ; i < joint_names_.size() ; i++)
	{
        joint_limits_[i] = p_s_->getRobotModel()->getJointModel(joint_names_[i])->getVariableBounds(joint_names_[i]);
//	 if(!p_s_->getRobotModel()->getJointModel(joint_names[i])->getVariableBounds(joint_names[i], joint_limits_[i]))
//		 std::cout <<" JOINT VARIABLE NOT FOUND !!!!!!  "<< std::endl;

	}


	//Init joint weights
	joint_weights_.resize(num_joints_);
	for (int i = 0; i < num_joints_; i++)
		joint_weights_[i] = 1.0; //all joints equally weighted


	//Set name of the start and goal tree
	T_start_.name = "start_tree";
	T_goal_.name = "goal_tree";

	//No articulation constraint active by default
	articulation_constraint_active_ = false;

	//Trajectory execution mode (only simulate trajectory -> traj_mode_= false / or execute trajectory -> traj_mode_= true)
	execute_trajectory_ = false;

	//Set velocity of hand for manipulation
	manipulation_velocity_ = 0.02; //in [m/s]
}


//Destructor
RRT_Planner::~RRT_Planner() {
	// TODO Auto-generated destructor stub
}




//Function to scale the support polygon (used for stat.-stability check)
void RRT_Planner::scale_support_polygon(float scale_sp)
{
	//Scale the support polygon
	stability_.scaleConvexHull(scale_sp);
}


//Set whether the trajectory is going to be executed (via MoveIt -> Trajectory Topic) or executed (via NaoQi -> angleInterpolation()-function)
void RRT_Planner::setTrajectoryMode(bool execute_trajectory)
{
	execute_trajectory_ = execute_trajectory;
}



//Load a set of statically stable configurations
void RRT_Planner::load_DS_database(char *stat_stable_config_file)
{
	// --------------- Get the number of configurations in the file --------------------------
	std::string line;
	num_configs_ = 0;
	std::ifstream config_file(stat_stable_config_file);
	if (config_file.is_open())
	{
		while ( config_file.good() )
		{
		 std::getline (config_file,line);
		 num_configs_++;
		}
		//Delete last line containing eof
		num_configs_--;
		std::cout <<"Set of " <<num_configs_ << " statically stable configurations used for sampling" << std::endl;
		config_file.close();
	}

	else std::cout << "Unable to open file (load DS database)";



	// --------------- Initialize Array storing the values from the file --------------------------
	ss_configs_=new double*[num_configs_]; //creates a new array of pointers to double objects
	for(int i=0; i<num_configs_; ++i)
	ss_configs_[i]=new double[num_joints_];



	//--------------- Fill the 2D Array --------------------------
	char * pEnd;
	int j = 0;
	char char_line[1000];
	config_file.open(stat_stable_config_file);
	if (config_file.is_open())
	{
		while ( config_file.good() )
		{
		 //read next line from file (either a configuration or eof)
		 std::getline (config_file,line);

		 //Detect when end of file is reached
		 if (config_file.eof())
			 break;

		 //Transform string into char array
		 strcpy(char_line,line.c_str());

		 //Char pointer pointing on first element of char array
		 char *tmp = char_line;

		 for (int i = 0 ; i < (num_joints_-1) ; ++i)
		 {
		   ss_configs_[j][i] = strtod(tmp,&pEnd);
		   //Assign the remaining char elements (pEnd) to the current char pointer
		   tmp = pEnd;
		 }
		 //Last joint value of configuration
		 ss_configs_[j][num_joints_-1] = strtod(tmp,NULL);

		 //Next configuration
		 j++;

		}
		config_file.close();
	}

	else std::cout << "Unable to open file (load DS database)";

}


//Activate a constraint for manipulating a drawer
void RRT_Planner::activate_drawer_constraint(std::vector<double> cart_pose_hand_start,std::vector<double> cart_pose_hand_goal, int num_interp_points, double translation_length)
{
	object_manipulation_.add_drawer_constraint(cart_pose_hand_start,cart_pose_hand_goal,num_interp_points);
	articulation_constraint_active_ = true;

	//Compute total time for manipulation in seconds
	float manipulation_time = (float)(translation_length) / manipulation_velocity_;

	//Timesteps for manipulation configurations (used when generating a manipulation trajectory)
	time_steps_manipulation_ = manipulation_time / float(num_interp_points + 2); //total time divided by (intermediate points + start and goal point)
}


//Activate a constraint for manipulating a door
void RRT_Planner::activate_door_constraint(std::vector<double> cart_pose_hand_start,std::vector<double> cart_pose_hand_goal, int num_interp_points, std::vector<double> door_param)
{
	object_manipulation_.add_door_constraint(cart_pose_hand_start,cart_pose_hand_goal,num_interp_points, door_param);
	articulation_constraint_active_ = true;

	//Compute the arc length (door_param[0] = door width , door_param[1] = door opening angle)
	float arc_length = float(door_param[0] * M_PI * door_param[1]) / 180.0;

	//Compute total time for manipulation in seconds
	float manipulation_time = arc_length / manipulation_velocity_;

	//Timesteps for manipulation configurations (used when generating a manipulation trajectory)
	time_steps_manipulation_ = manipulation_time / float(num_interp_points + 2); //total time divided by (intermediate points + start and goal point)
}



//Set the Start and the Goal Node
bool RRT_Planner::setStartGoalConfigs(std::vector<double> start_config , std::vector<double> goal_config)
{

    //std::cout<<"In setStartGoalConfigs function"<<std::endl;

	//Set the function to be called for stability checks
    //p_s_->setStateFeasibilityPredicate(boost::bind(&RRT_Planner::isFeasible, this, _1, _2));

	//Root of T_start
	Node start;
	start.index = 0;
	start.config = start_config;
	start.predecessor_index = 0;

	//Check if start config is valid (joint limits + stat.-stable + collision free)
	if (!isConfigValid(start))
	{
		std::cout<<"The start config is invalid !!!"<<std::endl;
		return false;
	}

	//	std::cout<<"The start config is : "<<std::endl;
	//	for (int t = 0 ; t< num_joints_ ; t++)
	//	std::cout<< start.config[t] << std::endl;



	//Root of T_goal
	Node goal;
	goal.index = 0;
	goal.config = goal_config;
	goal.predecessor_index = 0;

	//Check if start config is valid (joint limits + stat.-stable + collision free)
	if (!isConfigValid(goal))
	{
		std::cout<<"The goal config is invalid !!!"<<std::endl;
		return false;
	}


	//	std::cout<<"The goal config is : "<<std::endl;
	//	for (int t = 0 ; t< num_joints_ ; t++)
	//	std::cout<< goal.config[t] << std::endl;


	//If a consrtaints for articulated object manipulation is active
	if (articulation_constraint_active_ == true)
	{
		//Start cartesian hand pose index always zero
		start.cart_hand_pose_index = 0;
		//Index of goal cartesian hand pose
		goal.cart_hand_pose_index = (object_manipulation_.get_num_trajectory_points() - 1);

		//std::cout<<"Goal hand pos index" <<goal.cart_hand_pose_index<< std::endl;
//		//Test FK r foot to r arm
//		Node test_node;
//		Node q_near_test;
//		q_near_test.cart_hand_pose_index = (object_manipulation_.get_num_trajectory_points() - 1);
//		enforce_MA_Constraints("tree", q_near_test ,goal, test_node);

	}

	//Insert Root Nodes into Trees
	T_start_.nodes.push_back(start);
	T_start_.num_nodes = 1;
	T_goal_.nodes.push_back(goal);
	T_goal_.num_nodes  = 1;

	//if both configs are valid

    std::cout<<"Start and Goal Configuration is valid and set for planning"<<std::endl;

	return true;

}



//Set the joint weights (allows to disable joints for planning/ to move specific joints with preference)
void RRT_Planner::setJointWeights(std::vector<double> jw)
{
	for (int i = 0; i < num_joints_; i++)
			joint_weights_[i] = jw[i];
}



//Get a random stat.-stable sample
void RRT_Planner::getRandomStableConfig(Node &sample)
{
	//Select a random configuration from the goal configurations file
	int lb = 0, ub = num_configs_ - 1;
	int  random_config_index = floor(rng_.uniformReal(lb, ub));

	std::vector<double> tmp(num_joints_);
	for (int i = 0 ; i < num_joints_; i++)
	{
		tmp[i] = ss_configs_[random_config_index][i];
		//std::cout << tmp[i] << std::endl;
	}

	sample.config = tmp;


}


//Find the nearest neighbor for a given q_rand
void RRT_Planner::findNearestNeighbour(Tree T_current, Node q_rand, Node& nearest_neighbor)
{

	//Index of current nearest neighbor (-1 = no node)
	int nn_index = -1;
	std::vector<double> distance(num_joints_);
	//Norm of the distance
	double dist_norm = 0.0;
	//Sum of the squares
	double sum_sqrt = 0;
	//Init best norm found
	double best_norm = 10000.0;



	//Plot how many nodes are currently in the Tree
	//std::cout <<"Tree contains currently: "<<T_current.num_nodes << " nodes"<< std::endl;


	//Compute the distances between the q_rand node and the other nodes of the tree
	for (int i = 0; i < T_current.num_nodes; i++)
	{
		//Compute the euclidean distance between two nodes/configurations
		for (int n = 0 ; n < num_joints_ ; n++)
		{
			distance[n] = q_rand.config[n] - T_current.nodes[i].config[n];
			//std::cout <<distance[i]<< std::endl;
		}

		//Sum of squares
		sum_sqrt = 0;
		for (int j = 0 ; j < num_joints_ ; j++)
		{
			sum_sqrt = sum_sqrt + (distance[j]*distance[j]);
		}


		dist_norm = sqrt(sum_sqrt);
		//std::cout <<"Dist_norm is : " <<dist_norm<< std::endl;

		//If current norm is lower than the best update the best_norm and the nearest neighbor found
		if (dist_norm < best_norm)
		{
			best_norm = dist_norm;
			nn_index = i;
			//std::cout <<"Index set to: "<< i <<std::endl;
		}

	}

	//Check if a neighbor has been found
	if (nn_index < 0)
	{
		std::cout << "ERROR : No nearest neighbor found !!!";
	}

	else
	{
		//std::cout << "Nearest neighbor found, index is : "<<nn_index << std::endl;

		//Assign index of q_near found in the Tree to nearest_neighbor
		nearest_neighbor.index = T_current.nodes[nn_index].index;

		//Set the predecessor_index
		nearest_neighbor.predecessor_index = T_current.nodes[nn_index].predecessor_index;


		//Set the configuration of nearest_neighbor
		nearest_neighbor.config =  T_current.nodes[nn_index].config;

		//Set the cart_hand_pose_index of the nearest neighbor (only used for manipulation of articulated ojects)
		if (articulation_constraint_active_ == true)
		nearest_neighbor.cart_hand_pose_index =  T_current.nodes[nn_index].cart_hand_pose_index;

	}

	//Check nearest neighbor found
//	std::cout << "Nearest Neighbor is :" << std::endl;
//	for (int n = 0 ; n < num_joints_ ; n++)
//	{
//			std::cout << nearest_neighbor.config[n] << std::endl;
//	}

}




Status RRT_Planner::generate_q_new (Node q_near,Node q_rand, Node &q_new,  double max_step_size)
{
	//Status of Tree Extension
	Status status = TRAPPED;

	//Compute distance between the two configurations and its norm
	std::vector<double> distance(num_joints_);
	double sum_sqrt = 0.0;
	double dist_norm = 0.0;
	//Compute the euclidean distance between two nodes/configurations
	for (int n = 0 ; n < num_joints_ ; n++)
	{
		//Distance between the n-th component of q_rand and q_near
		distance[n] = q_rand.config[n] - q_near.config[n];
		//Sum of squares
		sum_sqrt = sum_sqrt + (distance[n]*distance[n]);
	}
	//Norm of distance vector q_near -> q_rand
	dist_norm = sqrt(sum_sqrt);


	//Compute unit vector of direction and the increment from q_near to q_new
	std::vector<double> direction(num_joints_);
	std::vector<double> increment(num_joints_);
	double sum_sqrt_incr = 0.0;
	double dist_norm_incr = 0.0;

	for (int j = 0 ; j < num_joints_ ; j++)
	{
		//Direction (unit vector)
		direction[j] = distance[j] / dist_norm;
		//Increment vector (joint increment depends strongly on joint weight)
		increment[j] = max_step_size * (direction[j]*joint_weights_[j]);
		//std::cout<<"Increment "<<j <<": " <<increment[j] << std::endl;

		//Sum of squares
		sum_sqrt_incr = sum_sqrt_incr + (increment[j]*increment[j]);
	}
	//Norm of distance vector q_near -> q_new
	dist_norm_incr = sqrt(sum_sqrt_incr);

	//std::cout<< "Distance q_rand - q_near: "<< dist_norm << std::endl;
	//std::cout<< "Distance q_rand - q_new: "<< dist_norm_incr << std::endl;


	//Compute q_new
	std::vector<double> tmp(num_joints_);
	if (dist_norm_incr < dist_norm)
	{
		//std::cout<< "q_new is generated...."<< std::endl;
		for (int n = 0 ; n < num_joints_ ; n++)
		{
			tmp[n] = q_near.config[n] + increment[n];
			//std::cout<<"Config element "<<n <<": "<< tmp[n] << std::endl;
		}
		q_new.config = tmp;


		status = ADVANCED;
	}
	else
	{
		q_new.config = q_rand.config;
		std::cout<< "q_new is q_rand !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<< std::endl;
		status = REACHED;
	}


	return status;

}


//Modify q_new such that the returned node obeys the constraints (No Collision, Stat.-Stable,....)
Status RRT_Planner::enforce_DS_Constraints(Node q_new, Node &q_new_modified)
{
	// Return value
	Status success = REACHED;

	//Check whether the new whole body configuration respects the joint limits
	bool joint_limits_respected = false;
	joint_limits_respected = local_planner_.checkJointLimits(q_new.config,joint_limits_);


	//Check whether the new config obeys the double support constraint
	if (joint_limits_respected == true)
    {
		//--------------------Get the configuration of the legs
        //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(group_name_)->getJointModelNames();
		//Right leg config
		std::vector<double> legs_config(10);
		for (std::size_t i = 0 ; i < 5 ; i++)
		{
			//std::cout << joint_names[i] << " : ";
			//Take the negative values of the right_leg configuration (because our kin. chain starts from foot and goes to the hip)
			legs_config[i] = -q_new.config[i];
			//std::cout << legs_config[i] << std::endl;

		}
		//Left leg config
		for (int i = 16 ; i < num_joints_ ; i++)
		{
			//std::cout << joint_names[i] << " : ";
			//Take the values of the left_leg configuration (from hip to left foot)
			legs_config[i-11] = q_new.config[i];
			//std::cout << legs_config[i-11] << std::endl;
		}


	  //--------------------------Compute the FK for the leg chain and check is double support is achieved
	  bool double_support = false;
	  double_support = local_planner_.checkLeftLegPose(legs_config);

	  //---------------------------When the current config is not in double support modify config of the left foot
	  if (double_support == false)
		  {
			//Get the limits for the joints of the left leg
            //std::vector< std::pair< double, double > > bounds_left_leg(5);
            std::vector<moveit::core::VariableBounds> bounds_left_leg(5);
			for (std::size_t i = 16 ; i < joint_limits_.size() ; ++i)
			{
                bounds_left_leg[i-16].min_position_ = joint_limits_[i].min_position_;
                bounds_left_leg[i-16].max_position_ = joint_limits_[i].max_position_;
			}

		   //Compute the IK for the left leg chain in order to guarantee double support
		   std::vector<double> new_l_leg_config(5); //Array storing the new config for the left leg
		   bool ik_solution_found = false;
		   ik_solution_found = local_planner_.computeLeftLegIK(legs_config, new_l_leg_config, bounds_left_leg);

		   //If a new config for the left leg satisfying the joints limits has been found
		   if (ik_solution_found == true)
		   {
			   //Assign current q_new to q_new_modified
			   q_new_modified.config = q_new.config;

			   //Assign new configuration to left leg
			   for (int i = 16 ; i < num_joints_ ; i++)
					{
					  q_new_modified.config[i] = new_l_leg_config[i-16];
					}
			   //State has been modified -> set the status to advanced
			   success = ADVANCED;
		   }
		   else
			   //std::cout<<"No IK solution found!!!" <<std::endl;
		       success = TRAPPED;

		  }

	  //q_new already satisfies the constraints
	  else
	  	  {
		    //Assign current q_new to q_new_modified
		  	q_new_modified.config = q_new.config;
	  	  }

    }


	else
	{
		  std::cout<<"q_new violates joint limits !!!" <<std::endl;
		  success = TRAPPED;
	}


    return success;

}


//Modify q_new_modified such that the returned node obeys the manipulation constraints (hand moves along line or circle)
Status RRT_Planner::enforce_MA_Constraints(std::string tree_name, Node q_near,Node q_new_ds, Node &q_new_manip)
{
	//Remark:
	// tree_name: tells us whether to use increased or decreased cart_hand_pose_index for right arm IK computation
	// q_near: provides index of cartesian hand pose
	// q_new_ds: the configuration generated by enforce_DS_Constraints (feet in double support)
	// q_new_manip: the new config produced by enforce_MA_Constraints (only differs in the right arm config when IK is computed)

	//Decide at which cartesian pose the hand should be for q_new
	int desired_cart_hand_pos_index = 0;
	if (tree_name == "start_tree" && q_near.cart_hand_pose_index <= (object_manipulation_.get_num_trajectory_points() - 2))
	desired_cart_hand_pos_index = q_near.cart_hand_pose_index + 1;
	else if (tree_name == "goal_tree" && q_near.cart_hand_pose_index > 0)
	desired_cart_hand_pos_index = q_near.cart_hand_pose_index - 1;
	else
	desired_cart_hand_pos_index = q_near.cart_hand_pose_index;


	std::cout <<"tree expanded: " <<tree_name <<std::endl;
	std::cout <<"hand pose index of q_near: " <<q_near.cart_hand_pose_index <<std::endl;
	std::cout <<"desired hand pose index for q_new: " <<desired_cart_hand_pos_index <<std::endl;


	//Get right leg configuration of q_new_ds
	std::vector<double> right_leg_config(5);
	for (std::size_t i = 0 ; i < 5 ; i++)
	{
		//Take the negative values of the right_leg configuration (because our kin. chain starts from foot and goes to the hip)
		right_leg_config[i] = -q_new_ds.config[i];
	}
	//Get right arm configuration of q_new_ds
	std::vector<double> right_arm_config(5);
	for (std::size_t i = 10 ; i < 15 ; i++)
		right_arm_config[i-10] = q_new_ds.config[i];


//	std::cout<<"Node index" <<q_near.index<<std::endl;
//	std::cout<<"Hand cart pos index" <<q_near.cart_hand_pose_index<<std::endl;

	//Check if q_new already has it's hand at the right cartesian pose
	bool correct_right_hand_pose = false;
	correct_right_hand_pose = object_manipulation_.right_hand_at_pose(desired_cart_hand_pos_index, right_leg_config, right_arm_config);


	//When hand is already at desired pose
	if(correct_right_hand_pose == true)
	{
		//Assign current q_new_ds to q_new_manip
		std::cout <<"Already at correct pose"<<std::endl;
		q_new_manip.config = q_new_ds.config;
		return REACHED;
	}
	else
	{
        std::cout <<"Right hand is not at correct pose"<<std::endl;

		//Get right arm configuration of q_new_ds
		std::vector<double> right_arm_config_reference(5);
		for (std::size_t i = 10 ; i < 15 ; i++)
			right_arm_config_reference[i-10] = q_near.config[i];

		//Get the limits for the joints of the right arm
        //std::vector< std::pair< double, double > > bounds_right_arm(5);
        std::vector<moveit::core::VariableBounds> bounds_right_arm(5);
		for (std::size_t i = 10 ; i < 15 ; ++i)
		{
            bounds_right_arm[i-10].min_position_ = joint_limits_[i].min_position_;
            bounds_right_arm[i-10].max_position_ = joint_limits_[i].max_position_;
		}

		//Compute the right arm IK for the desired hand pose
		bool right_hand_pose_reached = false;
		//Array in which the IK solution for the left leg will be stored
		std::vector<double> new_right_arm_config(5);
		right_hand_pose_reached = object_manipulation_.compute_right_arm_IK(desired_cart_hand_pos_index, right_leg_config, right_arm_config_reference, bounds_right_arm,new_right_arm_config);


		//If an IK solution has been found we return ADVANCED
		if(right_hand_pose_reached == true)
		{
            std::cout <<"A new configuration for the right arm to reach desired pose has been found"<<std::endl;
			//Assign current q_new_ds to q_new_manip
			q_new_manip.config = q_new_ds.config;

		   //Assign new configuration to right arm
		   for (int i = 10 ; i < 15 ; i++)
			{
			   q_new_manip.config[i] = new_right_arm_config[i-10];
			}

			return ADVANCED;
		}
		else
		{
            std::cout <<"No new configuration for the right arm to reach desired pose has been found"<<std::endl;
			return TRAPPED;
		}
	}

}



//Check whether q_new_modified is valid (No Collisions + Stat.-Stable)
bool RRT_Planner::isConfigValid(Node q_new_modified)
{

    //std::cout<<"In isConfigValid function"<<std::endl;

	//Map storing the configuration of the whole body
	std::map<std::string, double> configuration;
	for (int i = 0; i < num_joints_ ; i++)
	{
        configuration[joint_names_[i]] = q_new_modified.config[i];
        //std::cout<<joint_names_[i] <<": "<<configuration[joint_names_[i]]<<std::endl;
	}

	//Assign the robot state to the Kinematic Model
    robot_state::RobotState state(p_s_->getRobotModel());
    state.setToDefaultValues();

    //Set Joint Values
    //state.setStateValues(configuration);
    state.setVariablePositions(configuration);


	//Check collision info
    //bool colliding = false;
    //colliding = p_s_->isStateColliding(state,true);
    //std::cout<<"Is state in collision: "<<colliding << std::endl;


	//Check is state is statically stable and collision free
	bool isValid = false;
    //isValid = p_s_->isStateValid(state,group_name_,true);
    isValid = isFeasible(state,true);


	if (!isValid)
	{
        std::cout<<" STATE EITHER IN COLLISION or INSTABLE " << std::endl;
	}
	else
	{
        std::cout<<" STATE IS COLL.-FREE and STAT. STABLE " << std::endl;
	}

	return isValid;
}



void RRT_Planner::addConfigtoTree(Tree& tree, Node q_near , Node q_new_modified)
{
	//Set the index of the new node
	q_new_modified.index = tree.nodes.size();
	//std::cout << "Node with index: " << q_new_modified.index << " added"<< std::endl;

	//Set the index of the predecessor node (q_near)
	q_new_modified.predecessor_index = q_near.index;

	//------- Only required for manipulation of articulated objects
	if (articulation_constraint_active_ == true)
	{
		if (tree.name == "start_tree" && q_near.cart_hand_pose_index < (object_manipulation_.get_num_trajectory_points() - 2))
		q_new_modified.cart_hand_pose_index = q_near.cart_hand_pose_index + 1;
		else if (tree.name == "goal_tree" && q_near.cart_hand_pose_index > 0)
		q_new_modified.cart_hand_pose_index = q_near.cart_hand_pose_index - 1;
		else
		q_new_modified.cart_hand_pose_index = q_near.cart_hand_pose_index;


		//std::cout<<tree.name<<" "<<q_near.cart_hand_pose_index<<"  "<<q_new_modified.cart_hand_pose_index<<std::endl;
	}
	//-----------------------------------------------------

	//Add the new node to the tree
	tree.nodes.push_back(q_new_modified);

	//Increment number of nodes contained in the tree
	tree.num_nodes = tree.nodes.size();





}


//extend the tree (finds a valid q_new and adds it to the tree)
Status RRT_Planner::extendTree(Tree& tree, Node q_rand, Node& q_near, double max_step_size)
{
	//Node representing q_new
	Node q_new;
	//q_new after satisfying the double support constraint
	Node q_new_modified;
	//q_new after satisfying the manipulation constraint
	Node q_new_manipulate;

	//Tree state for q_new
	Status tree_state;
	//Tree state for q_new_modified
	Status tree_state_DS_constraint;
	//Tree state for q_new_manipulate
	Status tree_state_MA_constraint;

	//Find in T_first the nearest configuration to q_rand
	findNearestNeighbour(tree, q_rand, q_near);

	//Generate a q_new along the line connecting q_near to q_rand
	tree_state = generate_q_new (q_near, q_rand, q_new, max_step_size);


	if (tree_state != TRAPPED)
	{
		//Check if q_new obeys the constraints (respects joint limits + in double support)
		tree_state_DS_constraint = enforce_DS_Constraints(q_new, q_new_modified);

		if (tree_state_DS_constraint != TRAPPED)
		{
			if (tree_state_DS_constraint == ADVANCED)
				tree_state = ADVANCED;

			//TODO
			//here tree_state_constraint is REACHED or ADVANCED
			//if handle of drawer is attached to hand:
			//the new function sets the return value initially to REACHED
			//check if right hand is on a point on the line along the opening direction of the drawer -> if yes return REACHED
			//if the hand is not on the line try to find IK for right arm to bring right hand on the line
			// if IK succeeds, modify the right arm configuration and return ADVANCED
			// if IK fails, return TRAPPED
			bool proceed = true;
			if (articulation_constraint_active_ == true)
			{
				tree_state_MA_constraint = enforce_MA_Constraints(tree.name, q_near, q_new_modified, q_new_manipulate);
				//If right arm config has changed
				if (tree_state_MA_constraint == TRAPPED)
					proceed = false;
				if (tree_state_MA_constraint == ADVANCED)
					tree_state = ADVANCED;
			}

			else
			{
				//No changes applied to q_new obtained from enforce_DS_Constraints
				q_new_manipulate = q_new_modified;
			}

			//When manipulation constraint is satisfied or inactive
			if (proceed == true)
			{
				//Check if q_new_modified obeys the constraints (Collision free + Stat.-stable)
				bool hard_constraints_satisfied = false;
				hard_constraints_satisfied = isConfigValid(q_new_manipulate);


				if (hard_constraints_satisfied == true)
					{
					  //std::cout << "New config is valid" << std::endl;
					  //add configuration to tree
					  addConfigtoTree(tree, q_near, q_new_manipulate);
					}

				else
					{
					 //std::cout << "New config not valid" << std::endl;
					 tree_state = TRAPPED;
					}
			}
			else
			{
			 //std::cout << "Manipulation trapped" << std::endl;
			 tree_state = TRAPPED;
			}

		}


		else
		{
			tree_state = TRAPPED;
			//std::cout << "First constraints are not satisfied" << std::endl;
		}

	}

	return tree_state;
}




//extend the tree (finds a valid q_new and adds it to the tree)
Status RRT_Planner::connectTree(Tree& tree, Node q_connect, Node& q_near, double max_step_size)
{
	//Node representing q_new
	Node q_new;
	//q_new after satisfying the double support constraint
	Node q_new_modified;
	//q_new after satisfying the manipulation constraint
	Node q_new_manipulate;

	//Tree state for q_new
	Status tree_state = ADVANCED;
	//Tree state for q_new_modified
	Status tree_state_DS_constraint;
	//Tree state for q_new_manipulate
	Status tree_state_MA_constraint;


	//Find in T_first the nearest configuration to q_rand
	findNearestNeighbour(tree, q_connect, q_near);

	//Variable copy of q_near
	Node current_q_near;
	//Initialize q_near node (will change when performing multiple expansion steps)
	current_q_near = q_near;

	//If the cart_hand_pose_index of the q_near node in the start tree is bigger than the
	//cart_hand_pose_index of the last node q_connect of the goal tree
	if (articulation_constraint_active_ == true && tree.name == "start_tree" && q_near.cart_hand_pose_index > q_connect.cart_hand_pose_index)
		return TRAPPED;
	//If the cart_hand_pose_index of the q_near node in the goal tree is smaller than the
	//cart_hand_pose_index of the last node q_connect of the start tree
	if (articulation_constraint_active_ == true && tree.name == "goal_tree" && q_near.cart_hand_pose_index < q_connect.cart_hand_pose_index)
			return TRAPPED;


	//Expand tree until q_connect is reached or an invalid configurations has been encountered
	while (tree_state != TRAPPED && tree_state != REACHED)
	{
		std::cout<<"In connect Tree"<<std::endl;

		//Generate a q_new along the line connecting q_near to q_connect
		tree_state = generate_q_new (current_q_near, q_connect, q_new, max_step_size);

		if (tree_state != TRAPPED)
		{
			//Check if q_new obeys the constraints (respects joint limits + in double support)
			tree_state_DS_constraint = enforce_DS_Constraints(q_new, q_new_modified);

			if (tree_state_DS_constraint != TRAPPED)
			{
				if (tree_state_DS_constraint == ADVANCED)
					tree_state = ADVANCED;

				//TODO
				//here tree_state_constraint is REACHED or ADVANCED
				//if handle of drawer is attached to hand:
				//the new function sets the return value initially to REACHED
				//check if right hand is on a point on the line along the opening direction of the drawer -> if yes return REACHED
				//if the hand is not on the line try to find IK for right arm to bring right hand on the line
				// if IK succeeds, modify the right arm configuration and return ADVANCED
				// if IK fails, return TRAPPED
				bool proceed = true;
				if (articulation_constraint_active_ == true)
				{
					tree_state_MA_constraint = enforce_MA_Constraints(tree.name, current_q_near, q_new_modified, q_new_manipulate);
					//If no IK solution for the next cart_hand_pose_index has been found
					if (tree_state_MA_constraint == TRAPPED)
					{
						std::cout<<"TRAPPED"<<std::endl;
						proceed = false;
					}
					//If right arm config has changed
					if (tree_state_MA_constraint == ADVANCED)
					{
						std::cout<<"ADVANCED"<<std::endl;
						tree_state = ADVANCED;
					}
					//If q_connect has not been reached but the hand is already at the cart. pose of q_connect
				    if(current_q_near.cart_hand_pose_index == q_connect.cart_hand_pose_index && tree_state != REACHED)
				    {
				    	std::cout<<"TRAPPED, because hand_pose_index of q_new is equal to q_connect but q_connect has not been reached"<<std::endl;
				    	proceed = false;
				    }
				}

				else
				{
					//No changes applied to q_new obtained from enforce_DS_Constraints
					q_new_manipulate = q_new_modified;
				}

				//When manipulation constraint is satisfied or inactive
				if (proceed == true)
				{
					//Check if q_new_modified obeys the constraints (Collision free + Stat.-stable)
					bool hard_constraints_satisfied = false;
					hard_constraints_satisfied = isConfigValid(q_new_manipulate);


					if (hard_constraints_satisfied == true)
						{
						  std::cout << "New config is valid" << std::endl;

						  //Pause program to check output
						  //std::string dummy;
						  //std::getline(std::cin, dummy);

						  //add configuration to tree
						  addConfigtoTree(tree, current_q_near, q_new_manipulate);
						  //Set q_near node that has connected to the other tree
						  if(tree_state == REACHED)
							  q_near = current_q_near;
						  //Update current_q_near node to the last node added to the tree
						  else
							  current_q_near = tree.nodes.back();
						}

					else
						{
						 std::cout << "New config not valid" << std::endl;
						 tree_state = TRAPPED;
						}
				}
				else
				{
				 std::cout << "Manipulation trapped" << std::endl;
				 tree_state = TRAPPED;
				}

			}


			else
			{
				tree_state = TRAPPED;
				//std::cout << "First constraints are not satisfied" << std::endl;
			}

		}

		std::cout<<std::endl<<std::endl;
	}

	return tree_state;
}



//FUNCTION that solves a motion query
moveit_msgs::DisplayTrajectory RRT_Planner::solveQuery(int max_iter, double max_step_size, char* solution_file_path, int &num_nodes_generated , float &time_elapsed)
{
	//Object to be returned in case of success
	moveit_msgs::DisplayTrajectory solution_trajectory;

	//Init number of nodes generated
	num_nodes_generated = 0;

	//Flag to swap the trees
	bool swap_trees = false;

	//Random stat.-stable configuration
	Node q_rand;

	//Nearest Neighbor to q_rand (required in order to know which node is connected to the other tree in the final step)
	Node q_near;

	//Extend the tree
	Status returned;
	//Path found
	Status path_found;

	//Variable for timer (measuring time required to find a motion plan)
	timeval tim;
	gettimeofday(&tim, NULL);
	double t1=tim.tv_sec+(tim.tv_usec/1000000.0);

	//Iterate and expand the trees
	for (int i = 0 ; i < max_iter ; i++ )
	{
		std::cout<<"Iteration number: "<< i <<std::endl;
		//Get a random statically stable configuration
		getRandomStableConfig(q_rand);

		if (swap_trees == false)
		{
			returned = extendTree(T_start_,q_rand, q_near,max_step_size);

			//std::cout<<"The new config is : "<<std::endl;
			//for (int t = 0 ; t< num_joints_ ; t++)
			//std::cout<< T_start_.nodes.back().config[t] << std::endl;

			if (returned != TRAPPED)
			{
				//Try to connect the other tree to q_new previously generated
				//path_found = extendTree(T_goal_,T_start_.nodes.back(),q_near, max_step_size);
				path_found = connectTree(T_goal_,T_start_.nodes.back(),q_near, max_step_size);

				if (path_found == REACHED)
				{
					if (articulation_constraint_active_ == true)
						std::cout << T_goal_.nodes.back().cart_hand_pose_index<<" connected to "<<T_start_.nodes.back().cart_hand_pose_index << std::endl;

					std::cout << "PATH FOUND T_goal connected to T_start !!!!!!!!!!!" << std::endl;

					//Get time elapsed
					gettimeofday(&tim, NULL);
					double t2=tim.tv_sec+(tim.tv_usec/1000000.0);
					//printf("%.6lf seconds elapsed\n", t2-t1);
					time_elapsed = t2-t1;

					//Write solution path into a file
					writePath(T_start_, T_goal_, q_near, 1, solution_trajectory,solution_file_path);

					//Number of nodes generated in Start and Goal Tree
					std::cout<<"Number of nodes in the start tree : "<<T_start_.num_nodes <<std::endl;
					std::cout<<"Number of nodes in the goal tree : "<<T_goal_.num_nodes <<std::endl;

					//Store total number of nodes generated in referenced variable
					num_nodes_generated = T_start_.num_nodes + T_goal_.num_nodes;

					//Reset trees
					T_start_.nodes.clear();
					T_start_.num_nodes = 0;
					T_goal_.nodes.clear();
					T_goal_.num_nodes = 0;
					//Disable articulation constraint (for subsequent motion plans)
					articulation_constraint_active_ = false;


					//Exit from search
					return solution_trajectory;
				}
				//Otherwise swap the trees and proceed
				else
				{
				    swap_trees = true;
				}
			}
			//No Expansion of the first tree performed -> only swap trees
			else
			{
				swap_trees = true;
			}
		}

		//Start from goal tree
		else
		{
			returned = extendTree(T_goal_,q_rand, q_near, max_step_size);

			if (returned != TRAPPED)
			{
				//Try to connect the other tree to q_new previously generated
				//path_found = extendTree(T_start_,T_goal_.nodes.back(), q_near, max_step_size);
				path_found = connectTree(T_start_,T_goal_.nodes.back(), q_near, max_step_size);


				if (path_found == REACHED)
				{
					if (articulation_constraint_active_ == true)
					std::cout << T_start_.nodes.back().cart_hand_pose_index<<" connected to "<<T_goal_.nodes.back().cart_hand_pose_index << std::endl;

					std::cout << "PATH FOUND T_start connected to T_goal!!!!!!!!!!!" << std::endl;

					//Get time elapsed
					gettimeofday(&tim, NULL);
					double t2=tim.tv_sec+(tim.tv_usec/1000000.0);
					//printf("%.6lf seconds elapsed\n", t2-t1);
					time_elapsed = t2-t1;

					//Write solution path into a file
					writePath(T_start_, T_goal_, q_near, 2, solution_trajectory,solution_file_path);

					//Number of nodes generated in Start and Goal Tree
					std::cout<<"Number of nodes in the start tree : "<<T_start_.num_nodes <<std::endl;
					std::cout<<"Number of nodes in the goal tree : "<<T_goal_.num_nodes <<std::endl;

					//Store total number of nodes generated in referenced variable
					num_nodes_generated = T_start_.num_nodes + T_goal_.num_nodes;

					//Reset trees
					T_start_.nodes.clear();
					T_start_.num_nodes = 0;
					T_goal_.nodes.clear();
					T_goal_.num_nodes = 0;
					//Disable articulation constraint (for subsequent motion plans)
					articulation_constraint_active_ = false;


					//Exit from search
					return solution_trajectory;

				}
				//Otherwise swap the trees and proceed
				else
				{
					swap_trees = false;
				}
			}
			//No Expansion of the first tree performed -> only swap trees
			else
			{
				swap_trees = false;
			}

		}


	} //END of max iterations

	//Reset trees
	T_start_.nodes.clear();
	T_start_.num_nodes = 0;
	T_goal_.nodes.clear();
	T_goal_.num_nodes = 0;
	//Disable articulation constraint (for subsequent motion plans)
	articulation_constraint_active_ = false;


	//No solution path found
	return solution_trajectory;

}




//bool RRT_Planner::isFeasible(const robot_state::RobotState& state, bool verbose)
//{
bool RRT_Planner::isFeasible(robot_state::RobotState state, bool verbose)
{
    //std::cout << "In isFeasible function\n";

    //Get Joint Names
    std::vector<std::string> joint_names;
    joint_names = state.getVariableNames();

    //Get Joint Values
    const double *joint_values;
    joint_values = state.getVariablePositions();

    //Copy Joint names and values into map structure
    std::map< std::string, double > joint_state_values;
    for (int i = 0; i < joint_names.size(); i++)
    {
        joint_state_values.insert ( std::pair<std::string,double>(joint_names[i],joint_values[i]));
    }


    //Init check value
    bool statically_stable = false;
    //Init check value
    bool collision_free = false;

    //Flag indicating whether state is stat.-stable and coll.-free
    bool config_valid = false;


    // -------------------------------- Collision checking -------------------------------------
    //Collision checking Setup
    collision_detection::CollisionRequest collision_request;
    collision_request.group_name = group_name_;
    collision_detection::CollisionResult collision_result;
    collision_detection::AllowedCollisionMatrix acm = p_s_->getAllowedCollisionMatrix();
    //Clear the collision checking result
    collision_result.clear();
    //Check for collisions
    p_s_->checkCollision(collision_request, collision_result, state, acm);
    //Print collision checking results
    ROS_INFO_STREAM("Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

    if (collision_result.collision == 1)
    {
        collision_free = false;
        std::cout<< "State is in collision"<< std::endl;
    }
    else
    {
        collision_free = true;
        std::cout<< "State is collision-free"<< std::endl;
    }


    // -------------------------------- Stability checking ------------------------------------

    statically_stable = stability_.isPoseStable(joint_state_values,hrl_kinematics::TestStability::SUPPORT_DOUBLE);

    std::cout<< "Statically Stable : "<<statically_stable<< std::endl;


    //----------------------------- Return Final Result
    if (collision_free == true && statically_stable == true)
        config_valid = true;
    else
        config_valid = false;

    return config_valid;


//	//Plot joint values
//	//for (std::map<std::string,double>::const_iterator i = joint_state_values.begin(); i != joint_state_values.end(); ++i)
//	//	std::cout << i->first << ": " << i->second << std::endl;
//	bool statically_stable = false;

//	statically_stable = stability_.isPoseStable(joint_state_values,hrl_kinematics::TestStability::SUPPORT_DOUBLE);

//	//std::cout<< "Statically Stable : "<<statically_stable<< std::endl;

//	return statically_stable;
}



void RRT_Planner::writePath(Tree T_start, Tree T_goal, Node q_near, int connector, moveit_msgs::DisplayTrajectory &trajectory, char *solution_file_path)
{
	//std::cout<<"START WRITING FILE"<<std::endl;

	//Array containing the start path
	std::vector< std::vector<double> > start_path;
	//Array containing the goal path
	std::vector< std::vector<double> > goal_path;

	//Index of bridge nodes in start and goal tree
	int index_start = 0;
	int index_goal = 0;

   //Goal tree has connected to start tree
   if (connector == 1)
   {
	index_start = T_start.nodes.back().index;
	index_goal = q_near.index;
   }
   //Start tree has connected to goal tree
   else if (connector == 2)
   {
	index_start = q_near.index;
   	index_goal = T_goal.nodes.back().index;
   }
   else
   {
	   std::cout<< "Wrong connector value!!!"<< std::endl;
   }


	//Loop to write path from bridge to start config
	while (index_start > 0)
	{
		if (articulation_constraint_active_ == true)
		std::cout<<T_start.nodes[index_start].cart_hand_pose_index<<std::endl;

		start_path.push_back(T_start.nodes[index_start].config);
		index_start = T_start.nodes[index_start].predecessor_index;
	}
	//Add root configuration to array
	start_path.push_back(T_start.nodes[index_start].config);

	if (articulation_constraint_active_ == true)
	std::cout<<T_start.nodes[index_start].cart_hand_pose_index<<std::endl;




	//Loop to write path from bridge to start config
	while (index_goal > 0)
	{
		if (articulation_constraint_active_ == true)
		std::cout<<T_goal.nodes[index_goal].cart_hand_pose_index<<std::endl;

		goal_path.push_back(T_goal.nodes[index_goal].config);
		index_goal = T_goal.nodes[index_goal].predecessor_index;
	}
	//Add root configuration to array
	goal_path.push_back(T_goal.nodes[index_goal].config);

	if (articulation_constraint_active_ == true)
	std::cout<<T_goal.nodes[index_goal].cart_hand_pose_index<<std::endl;


	//	std::cout << "The goal config is: "<< std::endl;
	//	for (int j = 0 ; j<num_joints_ ; j++)
	//	{
	//		std::cout << T_goal.nodes[0].config[j]<< std::endl;
	//	}
	//	std::cout <<std::endl;
	//	std::cout << "First config i goal tree: "<< std::endl;
	//	for (int j = 0 ; j<num_joints_ ; j++)
	//	{
	//		std::cout << goal_path[goal_path.size()-1][j]<< std::endl;
	//	}

	//-----------------------------------------------------------

	//Create raw solution path
	std::vector< std::vector<double> > solution_path_configs;
	//Write the start_path into the solution_path array
	for (int i = (start_path.size()-1) ; i>= 0 ; i--)
		solution_path_configs.push_back(start_path[i]);


	//Write the goal_path into the solution_path array
	for (unsigned int i = 0 ; i < goal_path.size() ; i++)
		solution_path_configs.push_back(goal_path[i]);


	//+++++++++++++++ PATH SHORTCUTTER ++++++++++++++++
	std::vector< std::vector<double> > shortcutted_path;
	int num_intermediate_waypoints_approach = 100;
	int num_intermediate_waypoints_manipulate = 4;

	if (articulation_constraint_active_ == false)
	{
		//Try to shortcut the raw solution path (thus reducing configs while maintaining validity of path)
		//Note: internally interpolates waypoints of shortcutted path if trajectory is for simulation
		bool shortcutting_succeded = false;
		shortcutting_succeded = path_shortcutter(solution_path_configs, shortcutted_path,num_intermediate_waypoints_approach);

	}
	else
	{
		//If trajectory is only for simulation -> interpolate points of solution_path
		if (execute_trajectory_ == false)
		{
			//Interpolate Configurations of path_tmp
			interpolateShortPathWaypoints(solution_path_configs,shortcutted_path,num_intermediate_waypoints_manipulate);
		}
		//If trajectory is for execution with angleInterpolation(NAO API) -> return waypoints of original solution_path
		else
		{
			shortcutted_path = solution_path_configs;
		}
	}

	//Generate a trajectory from the solution_path (trajectory is returned by reference)
	generateTrajectory(shortcutted_path,shortcutted_path.size(),trajectory);


	//-------------------- Write solution path into file
	//Remove the current solution file
	if( std::remove( solution_file_path ) != 0 )
	{
		std::cout<< "Error deleting file (writePath)" <<std::endl;
	}

	//Files to store configurations of interpolated solution path
	std::ofstream solution_path;
	//Open a file which will store the computed statically stable whole-body configurations
	solution_path.open(solution_file_path); // opens the file
	if( !solution_path)
	  { // file couldn't be opened
            std::cerr << "Error: solution path file could not be opened" << std::endl;
			exit(1);
	  }

	//Write the interpolated_path into the file
	for (unsigned int i = 0 ; i < shortcutted_path.size() ; i++)
	{
		for (int j = 0 ; j<num_joints_ ; j++)
			solution_path << shortcutted_path[i][j] <<" ";

		//Set cursor to next line (for next configuration)
		solution_path << std::endl;

	}

	//Close the file
	solution_path.close();
	//-------------------------------------------------------------



}



void RRT_Planner::generateTrajectory(std::vector< std::vector<double> > path,int num_configs, moveit_msgs::DisplayTrajectory &traj)
{
    //planning_scene_monitor::LockedPlanningSceneRW p_s_RW(psm_);

	//----------------------------------------- TRAJECTORY PREPARATION ---------------------------------------
    //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(group_name_)->getJointModelNames();

	//++++++++++++++ SETUP ROBOT STATE MESSAGE (Start State)+++++++++++++
	moveit_msgs::RobotState start_state;
	start_state.joint_state.name.resize(num_joints_+3);
	start_state.joint_state.position.resize(num_joints_+3);


	// ------------- Set joint names and values of start state
	for (int i = 0 ; i < 5; i++)
	{
		//Set Joint Name
        start_state.joint_state.name[i] = joint_names_[i];
		start_state.joint_state.position[i] =  path[0][i];
        //std::cout<<trajectory.joint_names_[i]<< std::endl;
	}

	start_state.joint_state.name[5] = "RHipYawPitch";
	start_state.joint_state.position[5] =  0.0;
	start_state.joint_state.name[6] = "HeadYaw";
    start_state.joint_state.position[6] =  0.0;
	start_state.joint_state.name[7] = "HeadPitch";
	start_state.joint_state.position[7] =  0.0;

	for (int i = 8 ; i < num_joints_+3; i++)
	{
		//Set Joint Name
        start_state.joint_state.name[i] = joint_names_[i-3];
		start_state.joint_state.position[i] =  path[0][i-3];
        //std::cout<<trajectory.joint_names_[i]<< std::endl;
	}


	start_state.multi_dof_joint_state.joint_names.push_back("world_joint");

    //start_state.multi_dof_joint_state.frame_ids.push_back("odom");  ??????????????????????????????????????????
    //start_state.multi_dof_joint_state.child_frame_ids.push_back("r_sole");  ??????????????????????????????????????????

	geometry_msgs::Pose pose;
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 1.0;

    //start_state.multi_dof_joint_state.poses.push_back(pose);  ??????????????????????????????????????????


	//++++++++++++++ SETUP JOINT TRAJECTORY MESSAGE +++++++++++++
	trajectory_msgs::JointTrajectory trajectory;
	trajectory.joint_names.resize(num_joints_);
	trajectory.points.resize(num_configs);

	//Set joint names
	for (int i = 0 ; i < num_joints_; i++)
	{
		//Set Joint Name
        trajectory.joint_names[i] = joint_names_[i];
	}

	//Set Joint Values
	ros::Time start_time_instance =  ros::Time::now();
	ros::Duration time_from_start(0.0);
	float increment = 2.0;
	for (int i = 0 ; i < num_configs; i++)
	{
		//Set size of positions array
		trajectory.points[i].positions.resize(num_joints_);
		for (int j = 0 ; j < num_joints_; j++)
		{
		  //Set value
		  trajectory.points[i].positions[j] =  path[i][j];
		  //trajectory.points[i].velocities
		  trajectory.points[i].time_from_start = ros::Duration(increment);
		}
		if(articulation_constraint_active_ == true)
		{
			//std::cout<< increment<<std::endl;
			increment = increment + time_steps_manipulation_;
		}
		else
		increment = increment + 1.0;

	}

	//set frame id
	trajectory.header.frame_id = "odom";


	//std::cout<<"Joint Trajectory: "<< std::endl;
	//std::cout<<trajectory<< std::endl;


	//++++++++++++++++++++ TRAJECTORY SMOOTHING +++++++++++++++++++++
	//Robot Trajectory object
	moveit_msgs::RobotTrajectory robot_trajectory;

	//If trajectory is for manipulation and executed on NAOqi -> leave the trajectory time instances as they are defined above
	if(execute_trajectory_ == true && articulation_constraint_active_ == true)
	{
		//Set first point time instance (required to be non Zero by NAO API "angleInterpolation")
		//trajectory.points[0].time_from_start = ros::Duration(2.0);

		//Set robot trajectory to the original trajectory (no modifications to config time stamps applied)
		robot_trajectory.joint_trajectory = trajectory;
	}
	else
	{
		//Output trajectory
        trajectory_msgs::JointTrajectory smooth_trajectory;
        smooth_trajectory.joint_names.resize(num_joints_);
        smooth_trajectory.points.resize(num_configs);


		//Joint limits message vector
		std::vector<moveit_msgs::JointLimits> joint_lim(num_joints_);

		for (int i = 0 ; i < num_joints_ ; i++)
		{
         joint_lim[i].joint_name = joint_names_[i];
		 joint_lim[i].has_position_limits = true;
         joint_lim[i].min_position = joint_limits_[i].min_position_;
         joint_lim[i].max_position = joint_limits_[i].max_position_;
		 joint_lim[i].has_velocity_limits = true;
		 joint_lim[i].max_velocity = 0.5; //safe: 0.3
		 joint_lim[i].has_acceleration_limits = true;
		 joint_lim[i].max_acceleration = 0.1; //safe: 0.05 
		}


        //Assign original trajectory to robot_trajectory
        robot_trajectory.joint_trajectory = trajectory;


        //Setup initial robot state
        //Map storing the configuration of the whole body
        std::map<std::string, double> start_configuration;
        for (int j = 0; j < num_joints_ ; j++)
        {
            start_configuration[joint_names_[j]] = robot_trajectory.joint_trajectory.points[0].positions[j];
            //std::cout<<joint_names_[i] <<": "<<configuration[joint_names_[i]]<<std::endl;
        }
        //Assign the robot state to the Kinematic Model
        robot_state::RobotState state_init(p_s_->getRobotModel());
        state_init.setToDefaultValues();

        //Set Joint Values
        //state.setStateValues(configuration);
        state_init.setVariablePositions(start_configuration);


        //Setup robot_trajectory::RobotTrajectory
        robot_trajectory::RobotTrajectory robot_traj(p_s_->getRobotModel(), group_name_);
        robot_traj.setRobotTrajectoryMsg(state_init,robot_trajectory);


        // TODO : INSERT JOINT LIMITS IN TRAJECTORY !!!!!!!!!!!!!!!!!!!!!!!


        //Iterative Parabolic Trajectory Smoother
        trajectory_processing::IterativeParabolicTimeParameterization traj_smoother1;
        traj_smoother1.computeTimeStamps(robot_traj);
        //traj_smoother1.computeTimeStamps(trajectory,smooth_trajectory,joint_lim);                                            //TODO : smooth_trajectory is currently empty !!!!!!!!!!!!!!

        //Get the moveit_msgs::RobotTrajectory message back after smoothing
        robot_traj.getRobotTrajectoryMsg(robot_trajectory);

        //Assign everything from robot_trajectory::RobotTrajectory object to trajectory_msgs::JointTrajectory object
        smooth_trajectory = robot_trajectory.joint_trajectory;

        std::cout<<"Number of waypoints in the robot trajectory: "<<robot_traj.getWayPointCount()<<std::endl;


		//After smoothing time instance of first config is set to 0 !!! -> bad for execution with angleInterpolation
		// Modify trajectory time instances -> add 2 seconds to all configs (required to be non Zero by NAO API "angleInterpolation")
//        for (unsigned int i = 0 ; i < smooth_trajectory.points.size() ; i++)
//        {
//            //smooth_trajectory.points[i].time_from_start = smooth_trajectory.points[i].time_from_start + ros::Duration(2.0);
//            //smooth_trajectory.points[i].time_from_start = robot_traj.joint_trajectory.points[i].time_from_start + ros::Duration(2.0);
//        }

        for (unsigned int i = 0 ; i < robot_trajectory.joint_trajectory.points.size() ; i++)
        {
            smooth_trajectory.points[i].time_from_start = robot_trajectory.joint_trajectory.points[i].time_from_start + ros::Duration(2.0);
            //smooth_trajectory.points[i].time_from_start = robot_traj.joint_trajectory.points[i].time_from_start + ros::Duration(2.0);
        }



		//Set robot trajectory to the smoothed version of trajectory
        robot_trajectory.joint_trajectory = smooth_trajectory;

	}


	//+++++++++++++ SETUP DISPLAY TRAJECTORY MESSAGE ++++++++++++
    traj.model_id = p_s_->getRobotModel()->getName();
	traj.trajectory_start = start_state;
    traj.trajectory.clear();
	traj.trajectory.push_back(robot_trajectory);


}


//Linear interpolation of two waypoints/configurations -> return value is a trajectory
moveit_msgs::DisplayTrajectory RRT_Planner::interpolateWaypoints(std::vector<double> waypoint_start,std::vector<double> waypoint_goal, int num_intermediate_waypoints)
{

    //planning_scene_monitor::LockedPlanningSceneRW p_s_RW(psm_);

	//Object to be returned in case of success
	moveit_msgs::DisplayTrajectory interpolation_trajectory;
	//Joint Trajectory
	trajectory_msgs::JointTrajectory joint_trajectory;
	//Robot Trajectory
	moveit_msgs::RobotTrajectory robot_trajectory;
	//Robot state
	moveit_msgs::RobotState start_state;

	//Joint names
    //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(group_name_)->getJointModelNames();


	//++++++++++++++ SETUP ROBOT STATE MESSAGE (Start State)+++++++++++++
	start_state.joint_state.name.resize(num_joints_+3);
	start_state.joint_state.position.resize(num_joints_+3);


	// ------------- Set joint names and values of start state
	for (int i = 0 ; i < 5; i++)
	{
		//Set Joint Name
        start_state.joint_state.name[i] = joint_names_[i];
		start_state.joint_state.position[i] =  waypoint_start[i];
        //std::cout<<trajectory.joint_names_[i]<< std::endl;
	}

	start_state.joint_state.name[5] = "RHipYawPitch";
	start_state.joint_state.position[5] =  0.0;
	start_state.joint_state.name[6] = "HeadYaw";
	start_state.joint_state.position[6] =  0.0;
	start_state.joint_state.name[7] = "HeadPitch";
	start_state.joint_state.position[7] =  0.0;

	for (int i = 8 ; i < num_joints_+3; i++)
	{
		//Set Joint Name
        start_state.joint_state.name[i] = joint_names_[i-3];
		start_state.joint_state.position[i] = waypoint_start[i-3];
        //std::cout<<trajectory.joint_names_[i]<< std::endl;
	}


	start_state.multi_dof_joint_state.joint_names.push_back("world_joint");

    //start_state.multi_dof_joint_state.frame_ids.push_back("odom");  ??????????????????????????????????????????
    //start_state.multi_dof_joint_state.child_frame_ids.push_back("r_sole");  ??????????????????????????????????????????

	geometry_msgs::Pose pose;
	pose.position.x = 0.0;
	pose.position.y = 0.0;
	pose.position.z = 0.0;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = 0.0;
	pose.orientation.w = 1.0;


    //start_state.multi_dof_joint_state.poses.push_back(pose);  ??????????????????????????????????????????
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


	//+++++++++++++ Interpolate Waypoints
	//Array of waypoints
	std::vector< std::vector<double> > waypoints_interpolated;

	//Get number of configs of the raw solution path
	//int raw_solution_num_waypoints = raw_solution_path_configs.size();

	//Step width
	std::vector<double> step_width(num_joints_);

	//compute step width
	for (int j = 0; j < num_joints_; j++)
	{
		//step_width[j] = (waypoint_start[j]- waypoint_goal[j]) / (num_intermediate_waypoints+1);
		step_width[j] = (waypoint_goal[j]- waypoint_start[j]) / (num_intermediate_waypoints+1);
		//std::cout<< "Step width: "<< step_width[j] << std::endl;
	}

	//First config of trajectory
	waypoints_interpolated.push_back(waypoint_start);

	//compute intermediate states
	std::vector<double> tmp(num_joints_);
	for (int n = 1; n <= num_intermediate_waypoints+1; n++)
	{
		//std::cout<< "State num: "<<row <<std::endl;
		for (int k = 0; k < num_joints_; k++)
			tmp[k] = waypoint_start[k] + (step_width[k] * n);

		waypoints_interpolated.push_back(tmp);
	}
	//++++++++++++++++++++++++++++++++++++++++++++


	//++++++++++++++ SETUP JOINT TRAJECTORY MESSAGE +++++++++++++
	joint_trajectory.joint_names.resize(num_joints_);
	joint_trajectory.points.resize(num_intermediate_waypoints+2); //intermediate points + start and goal waypoint

	//Set joint names
	for (int i = 0 ; i < num_joints_; i++)
	{
		//Set Joint Name
        joint_trajectory.joint_names[i] = joint_names_[i];
	}

	//Set Joint Values
	ros::Time start_time_instance =  ros::Time::now();
	ros::Duration time_from_start(0.0);
	float increment = 0.0;
	for (int i = 0 ; i < num_intermediate_waypoints+2; i++)
	{
		//Set size of positions array
		joint_trajectory.points[i].positions.resize(num_joints_);
		for (int j = 0 ; j < num_joints_; j++)
		{
		  //Set value
		  joint_trajectory.points[i].positions[j] =  waypoints_interpolated[i][j];
		  joint_trajectory.points[i].time_from_start = ros::Duration(increment);
		}
		increment = increment + 1.0;

	}

	//set frame id
	joint_trajectory.header.frame_id = "odom";


	//++++++++++++++ SETUP ROBOT TRAJECTORY MESSAGE +++++++++++++
	//Set the joint trajectory
	robot_trajectory.joint_trajectory = joint_trajectory;


	//+++++++++++++ SETUP DISPLAY TRAJECTORY MESSAGE ++++++++++++
    interpolation_trajectory.model_id = p_s_->getRobotModel()->getName();
	interpolation_trajectory.trajectory_start = start_state;
    interpolation_trajectory.trajectory.clear();
    interpolation_trajectory.trajectory.push_back(robot_trajectory);


	//Return Display Trajectory
	return interpolation_trajectory;
}



//Given the raw solution path tries to reduce the number of waypoints/configurations while maintaining validity of the path
bool RRT_Planner::path_shortcutter(std::vector< std::vector<double> > raw_solution_path, std::vector< std::vector<double> > &shortcutted_path, int num_intermediate_waypoints)
{

    //planning_scene_monitor::LockedPlanningSceneRW p_s_RW(psm_);

	//Temporary path (assigned to shortcutted_path if execute_trajectory_ == true)
	std::vector< std::vector<double> > path_tmp;

	//Insert start configuration into shortcutted_path
	path_tmp.push_back(raw_solution_path[0]);

	//Current start config (= first config of raw solution path)
	std::vector<double> current_start = raw_solution_path[0];
	int current_start_index = 0;

	//Current goal config (= last config of raw solution path)
	std::vector<double> current_goal;

	//Goal config (= last config of raw solution path)
	std::vector<double> goal = raw_solution_path[raw_solution_path.size()-1];


	//Init shortcutting state
	bool goal_config_reached = false;

	//Maximum number of iterations allowed to shortcut path (for security reasons, otherwise program gets eventually stuck)
	int max_iterations_allowed = 500;

    //Current Robot Trajectory (to be checked for validity)
    moveit_msgs::RobotTrajectory current_trajectory;

    //Shortcutter Loop
	while (goal_config_reached == false && max_iterations_allowed > 0)
	{
		std::cout<<"In Path Shortcutter...."<<std::endl;

		//Interpolated trajectory
		moveit_msgs::DisplayTrajectory waypoints_interpolated;
		//Interpolate the two configurations (return value is a trajectory from current_start to goal)
		waypoints_interpolated = interpolateWaypoints(current_start,goal,num_intermediate_waypoints);


        current_trajectory = waypoints_interpolated.trajectory.at(0);
		std::cout<<"goal check"<<std::endl;

		//Check if path to goal config is Valid
        if(p_s_->isPathValid(waypoints_interpolated.trajectory_start,current_trajectory))
		{
			path_tmp.push_back(goal);
			goal_config_reached = true;
		}
		else
		{
			//Go backwards from goal and try to find a valid path
			for (int i = raw_solution_path.size()-2 ; i >= 0 ; i--)
			{
                std::cout<<" Shortcut to Node: "<< i <<std::endl;
				//Set current goal config
				current_goal = raw_solution_path[i];

				//Check if current_start = current_goal -> in this case shorcutter is stuck
				if (current_start_index == i)
				{
					current_start = raw_solution_path[i+1];
					current_start_index = i+1;
					path_tmp.push_back(current_start);
					break;
				}

				//Interpolate the two configurations (return value is a trajectory from current_start to goal)
				waypoints_interpolated = interpolateWaypoints(current_start,current_goal,num_intermediate_waypoints);
                current_trajectory = waypoints_interpolated.trajectory.at(0);

                std::cout<<"goal check"<<std::endl;

                if(p_s_->isPathValid(waypoints_interpolated.trajectory_start,current_trajectory))
				{
					std::cout<<"new start"<<std::endl;

					current_start = current_goal;
					current_start_index = i;
					path_tmp.push_back(current_goal);
					//Leave loop when path to current_goal config is valid
					break;
				}

			} //END for loop
		} //END else section
		max_iterations_allowed--;
	} //END while loop


	//If Shortcutting failed
	if(max_iterations_allowed == 0)
	{
		return false;
	}

	else
	{

		//Output the number of waypoints of the shortcutted_path
		std::cout<<"Shortcutted Path has " <<path_tmp.size() <<" waypoints"<<std::endl;

		//If trajectory is only for simulation -> interpolate points of shortcutted path
		if (execute_trajectory_ == false)
		{
			//Interpolate Configurations of path_tmp
			interpolateShortPathWaypoints(path_tmp,shortcutted_path,num_intermediate_waypoints);
		}
		//If trajectory is for execution with angleInterpolation(NAO API) -> return waypoints of path_tmp
		else
		{
			shortcutted_path = path_tmp;
		}

		//If Shortcutting succeded
		return true;
	}


}



//Interpolate the waypoints of the shortcutted path
void RRT_Planner::interpolateShortPathWaypoints(std::vector<std::vector<double> > short_path, std::vector<std::vector<double> > &interpolated_path, int num_intermediate_waypoints)
{

	//Assign first config of shortcutted trajectory to interpolated_path
	interpolated_path.push_back(short_path[0]);

	//Step width
	std::vector<double> step_width(num_joints_);

	//Next intermediate waypoints
	std::vector<double> next_waypoint(num_joints_);

	//Linear interpolation
	for (unsigned int i = 0; i < short_path.size()-1; i++)
	{
		//compute step width
		for (int j = 0; j < num_joints_; j++)
		{
			step_width[j] = (short_path[i+1][j]-short_path[i][j]) / (num_intermediate_waypoints+1);
			//std::cout<< "Step width: "<< step_width[j] << std::endl;
		}

		//compute intermediate states
		for (int n = 1; n <= num_intermediate_waypoints+1; n++)
		{
			//std::cout<< "State num: "<<row <<std::endl;
			for (int k = 0; k < num_joints_; k++)
			{
				next_waypoint[k] = short_path[i][k] + (step_width[k] * n);
				//std::cout<< interpolated_path_[row][k] << std::endl;
			}
			//Insert new intermediate waypoint into interpolated_path
			interpolated_path.push_back(next_waypoint);
			//std::cout<< std::endl;
		}

	}

}



//Fix Joint name order from RobotModel to order used by planner
std::vector<std::string> RRT_Planner::joint_name_order_RobModel_to_Planner(const std::vector<std::string> j_names)
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

} //END OF NAMESPACE
