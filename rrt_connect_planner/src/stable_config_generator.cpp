/*
 * stable_config_generator.cpp
 *
 *  Created on: May 10, 2012
 *      Author: Felix Burget
 */

#include "rrt_connect_planner/stable_config_generator.h"



namespace nao_constraint_sampler {


//StableConfigGenerator::StableConfigGenerator(boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm ,const std::string &group_name, float scale_sp) : nh_("~"), psm_(psm),group_name_(group_name)
StableConfigGenerator::StableConfigGenerator(boost::shared_ptr<planning_scene::PlanningScene> p_s ,const std::string &group_name, float scale_sp) : nh_("~"), p_s_(p_s),group_name_(group_name)
{
  //-------------------------------------- RECONFIGURE THE Kin.Model---------- ----------------------------

  //planning_scene_monitor::LockedPlanningSceneRO p_s_RO(psm_);

  //Get the urdf and srdf model
  //const boost::shared_ptr< const urdf::ModelInterface > & urdf_nao = psm->getPlanningScene()->getRobotModel()->getURDF();
  const boost::shared_ptr< const urdf::ModelInterface > & urdf_nao =  p_s_->getRobotModel()->getURDF();


  //const boost::shared_ptr< const srdf::Model > & srdf_nao = psm->getPlanningScene()->getRobotModel()->getSRDF();
  const boost::shared_ptr< const srdf::Model > & srdf_nao =  p_s_->getRobotModel()->getSRDF();

  //Configure the planning scene (set root joint)
  //
//  if(!p_s_->configure(urdf_nao, srdf_nao, "r_sole"))
//  {
//	  std::cout<< "FAILED to configure planning scene!!!" << std::endl;
//  }


  //Get Joint Names of planning group
  const std::vector<std::string> joint_names = p_s_->getRobotModel()->getJointModelGroup(group_name_)->getJointModelNames();


  //psm_->unlockSceneWrite();
  //psm_->unlockSceneRead();

  //Fix the order of the joint names
  joint_names_ = joint_name_order_RobModel_to_Planner(joint_names);


  for (int i = 0; i< joint_names_.size(); i++)
  {
      std::cout<<"Joint name: "<<joint_names_[i]<<std::endl;
  }


  //Print the name of the root joint
  ROS_INFO("Root JOINT is %s ",p_s_->getRobotModel()->getRootJointName().c_str());
  ROS_INFO("Root LINK is %s ",p_s_->getRobotModel()->getRootLink()->getName().c_str());

  //Planning Scene
  //scene_ = psm_->getPlanningScene();

  //Scale the support polygon
  stability_.scaleConvexHull(scale_sp);

}



StableConfigGenerator::~StableConfigGenerator()
{
	// TODO Auto-generated destructor stub
}


//bool StableConfigGenerator::isFeasible(const robot_state::RobotState& state, bool verbose)
//{
bool StableConfigGenerator::isFeasible(robot_state::RobotState state, bool verbose)
{
    std::cout << "isFeasible reached\n";

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

    //Plot joint values
    //for (std::map<std::string,double>::const_iterator i = joint_state_values.begin(); i != joint_state_values.end(); ++i)
    //	std::cout << i->first << ": " << i->second << std::endl;

    //state.getStateValues(joint_state_values);

	//Plot joint values
	//for (std::map<std::string,double>::const_iterator i = joint_state_values.begin(); i != joint_state_values.end(); ++i)
	//	std::cout << i->first << ": " << i->second << std::endl;

    //Init check value
    bool statically_stable = false;
    //Init check value
    bool collision_free = false;

    //Flag indicating whether state is stat.-stable and coll.-free
    bool config_valid = false;


    std::cout << "SCG Before collision checking\n";

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


     std::cout << "SCG Before stability checking\n";

    // -------------------------------- Stability checking ------------------------------------

	statically_stable = stability_.isPoseStable(joint_state_values,hrl_kinematics::TestStability::SUPPORT_DOUBLE);

    std::cout<< "Statically Stable : "<<statically_stable<< std::endl;


    //----------------------------- Return Final Result
    if (collision_free == true && statically_stable == true)
        config_valid = true;
    else
        config_valid = false;

    return config_valid;

}





//Specify the desired position of the right hand (the orientation is such that the y axis of the hand and the r_foot are parallel)
void StableConfigGenerator::setDesiredRightArmPose(std::vector<double> right_hand_pose)
{
	leg_kinematics_.desiredRightArmPose(right_hand_pose);

}





int StableConfigGenerator::sample_stable_configs(int max_samples, int max_ik_iterations, float &time_elapsed,bool sample_goal_configs = false, char* file_destination = "config.dat")
{
    //planning_scene_monitor::LockedPlanningSceneRW p_s_RW(psm_);

	//Set the function to be called for stability checks
    //p_s_->setStateFeasibilityPredicate(boost::bind(&StableConfigGenerator::isFeasible, this, _1, _2));



    //----------------------------------------
    //psm_->unlockSceneWrite();
    //psm_->unlockSceneRead();
    //----------------------------------------

	//Remove the file with the old statically stable configs
    if( std::remove ( file_destination ) != 0 )
	{
        std::cout<< "Error deleting file (sample stable configs)" <<std::endl;
	}

	//---------------------------------- Open a file (either for stat.-stable or goal configs) -------------------------------------------
	//Open a file which will store the computed statically stable whole-body configurations
	if (sample_goal_configs == false)
	{
		stat_stable_samples_DS_.open(file_destination); // opens the file
		if( !stat_stable_samples_DS_ )
		{
			// file couldn't be opened
            std::cerr << "Error: file for stat_stable_samples_DS configs could not be opened" << std::endl;
			exit(1);
		}
	}


   //Open a file which will store the computed statically stable whole-body goal configurations (hand positioned at desired pose)
   if (sample_goal_configs == true)
   {
	   stat_stable_goal_samples_DS_.open(file_destination); // opens the file
       if( !stat_stable_goal_samples_DS_ )
	   {
			// file couldn't be opened
            std::cerr << "Error: file for stat_stable_goal_samples_DS configs could not be opened" << std::endl;
			exit(1);
	   }
   }

	//---------------------------------------- Get Joint Limits -----------------------------------------------------
	//Joint Names of the group
    //const std::vector<std::string>& joint_names = scene_->getRobotModel()->getJointModelGroup(group_name_)->getJointModelNames();


	//Get the limits for the joints in group "group_name_"
    //std::vector< std::pair< double, double > > bounds(joint_names.size());
    std::vector<moveit::core::VariableBounds> bounds(joint_names_.size());
    for(std::size_t i = 0 ; i < joint_names_.size() ; i++)
	{
        bounds[i] = p_s_->getRobotModel()->getJointModel(joint_names_[i])->getVariableBounds(joint_names_[i]);
        std::cout<<"Name: "<<joint_names_[i]<<"  Min. Position: " <<bounds[i].min_position_<<"  Max. Position: " <<bounds[i].max_position_<<std::endl;
     //if(!p_s_->getRobotModel()->getJointModel(joint_names_[i])->getVariableBounds(joint_names_[i], bounds[i]))
        // std::cout <<" JOINT VARIABLE NOT FOUND !!!!!!  "<< std::endl;

	}


	//Get the limits for the joints of the left leg
    //std::vector< std::pair< double, double > > bounds_left_leg(5);
    std::vector<moveit::core::VariableBounds> bounds_left_leg(5);
	int counter = 0;
	for (std::size_t i = 16 ; i < bounds.size() ; ++i)
	{
        bounds_left_leg[counter].min_position_ = bounds[i].min_position_;
        bounds_left_leg[counter].max_position_ = bounds[i].max_position_;
		counter++;

	}

	//Get the limits for the joints of the right arm
    //std::vector< std::pair< double, double > > bounds_right_arm(5);
    std::vector<moveit::core::VariableBounds> bounds_right_arm(5);
	int count = 0;
	for (std::size_t i = 10 ; i < 15 ; ++i)
	{
        bounds_right_arm[count].min_position_ = bounds[i].min_position_;
        bounds_right_arm[count].max_position_ = bounds[i].max_position_;
		count++;

	}


	//Count the number of statically stable configurations found
	int stat_stable_samples = 0;
	//Count the number of statically stable goal configurations found
	int goal_samples = 0;

	//Variable for timer (measuring time required to find a motion plan)
	bool first_goal_config = true;
	time_elapsed = 0.0;
	timeval tim;
	gettimeofday(&tim, NULL);
	double t1=tim.tv_sec+(tim.tv_usec/1000000.0);

	//Maximum Iterations to sample/collect statically stable double support poses
	for (int i = 0 ; i < max_samples ; i++)
	{

        //std::cout<<"Iteration SSC: "<<i<<std::endl;
		//------------------------------- Generate random configuration (values are within joint limits) ------------------------------------

		//Map storing a random configuration of the whole body
		std::map<std::string, double> joint_values;
		// enforce the constraints for the constrained components (could be all of them)
		for (std::size_t i = 0 ; i < bounds.size() ; ++i)
		{
            joint_values[joint_names_[i]] = random_number_generator_.uniformReal(bounds[i].min_position_, bounds[i].max_position_);
			//Keep the HipYawPitch joint fix (otherwise feet can't be parallel!!! -> HipYawPitch of R/L foot coupled!!!)
			joint_values["LHipYawPitch"] = 0.0;

			//Fix left arm : when we sample goal configurations we want to avoid strange left arm configs
			if (sample_goal_configs == true)
			{
				joint_values["LShoulderPitch"] = 1.5708;
				joint_values["LShoulderRoll"] = 0.17;
				joint_values["LElbowYaw"] = 0.0;
				joint_values["LElbowRoll"] = -0.036;
				joint_values["LWristYaw"] = 0.0;
			}
		}


		//--------------------------- Compute the pose of the Hip (right foot is assumed to be on the ground) -------------------------------
		//Get right leg configuration from joint_values array
		std::vector<double> right_leg_values(5);
		for (std::size_t i = 0 ; i < 5 ; ++i)
		{
			//Take the negative values of the right_leg configuration (because our kin. chain starts from foot and goes to the hip)
            right_leg_values[i] = -joint_values[joint_names_[i]];
            //std::cout << joint_names_[i] <<":   " <<right_leg_values[i]<< std::endl;
		}


		//Compute the pose of the hip
		KDL::Frame hip_pose;
		bool fk_rl_ok = false;
		fk_rl_ok = leg_kinematics_.getHipPose(right_leg_values,hip_pose);



		//----------------------------- Find IK for left foot (such that l_foot is on the ground, i.e parallel to r_foot) ------------------
		//Array in which the IK solution for the left leg will be stored
		std::vector<double> left_leg_values(5);

		//Computes the IK for the left leg, such that the left foot has always the same relative pos w.r.t the right foot
		bool double_support_achieved = false;
		if (fk_rl_ok >= 0)
        {
            double_support_achieved = leg_kinematics_.computeLeftLegIK(hip_pose,bounds_left_leg,left_leg_values,max_ik_iterations);
            //std::cout<<"Double support achieved: "<<double_support_achieved<<std::endl;
        }

		//-------------------------- Additionally try to reach goal pose with the arm (if sample_goal_configs set to true) -----------------
		//Array in which the IK solution for the left leg will be stored
		std::vector<double> right_arm_values(5);
		//Stores ergonomy of the arm posture found
		double ergonomics_index = 0.0;
		//Computes the IK for the right arm, such that the right arm reaches the grasp position
		bool grasp_position_reached = false;
		//Only perform right arm IK if we want to sample goal configs (i.e sample_goal_configs = true)
		if (double_support_achieved == true && sample_goal_configs == true)
        {
		  grasp_position_reached = leg_kinematics_.computeRightArmIK(hip_pose,bounds_right_arm,right_arm_values,ergonomics_index);
          std::cout<<"Grasp position reached output: "<<grasp_position_reached<<std::endl;
        }


		//------------------------ Assign new configuration to left leg and right arm (if a valid IK solution has been found) --------------

		//Assign new configuration (if found by IK !!!) for left leg to the joint_values array
		int count = 0;
		if (double_support_achieved == true)
		{
            for (std::size_t i = 16 ; i < joint_names_.size() ; ++i)
			{
                joint_values[joint_names_[i]] = left_leg_values[count];
                //std::cout << joint_names_[i] <<":   " <<joint_values[joint_names_[i]]<< std::endl;
				count++;
			}

			//Assign new configuration (if found by IK !!!) for right arm to the joint_values array
			int iterator = 0;
			if (grasp_position_reached == true)
			{
				for (std::size_t i = 10 ; i < 15 ; ++i)
					{
                      joint_values[joint_names_[i]] = right_arm_values[iterator];
                      //std::cout << joint_names_[i] <<":   " <<joint_values[joint_names_[i]]<< std::endl;
					  iterator++;
					}
			}
		}


		//------------------------------------ Check if whole-body configuration is valid (no self-collision + stat. stable) ----------------
		// We want to sample stat.-stable goal configs (a grasp_position for the right arm is required)
		if (double_support_achieved == true && grasp_position_reached == true)
		{
			//Assign the robot state to the Kinematic Model
            robot_state::RobotState state(p_s_->getRobotModel());
            state.setToDefaultValues();

            //Set Joint Values
            //state.setStateValues(joint_values);
            state.setVariablePositions(joint_values);

            //for (int i = 0 ; i<joint_names_.size() ;  i++)
            //{
            //    std::cout<<"Joint "<<joint_names_[i]<<" has value: "<< state.getVariablePosition(joint_names_[i])<<std::endl;
            //}
            

			//Check if the given state is valid. This means checking for collisions and feasibility (setStateFeasibilityPredicate set to isFeasible()-function).
            //if (!p_s_->isStateValid(state,group_name_,true))
            //{
            if (!isFeasible(state,true))
            {

                std::cout<<" STATE NOT FEASIBLE " << std::endl;
				continue;	//in case the configuration in invalid (self-collision) return and generate a new random config
			}
			else
			{
                std::cout<<" STATE FEASIBLE !!!" << std::endl;

				//Write the ergonomy index into the first column of the file
				stat_stable_goal_samples_DS_ << ergonomics_index <<" ";

				//Write the configuration into the file
                for (std::size_t i = 0 ; i<joint_names_.size() ; i++)
				{
                    stat_stable_goal_samples_DS_ << joint_values[joint_names_[i]] <<" ";
				}
				//Set cursor to next line (for next configuration)
				stat_stable_goal_samples_DS_ << std::endl;
				//Increment number of statically stable goal samples found
				goal_samples++;

				//Get time elapsed to find first goal config
				if(first_goal_config == true)
				{
					gettimeofday(&tim, NULL);
					double t2=tim.tv_sec+(tim.tv_usec/1000000.0);
					printf("%.6lf seconds elapsed\n", t2-t1);
					time_elapsed = t2-t1;
					first_goal_config = false;
				}

				//Leave loop when more than 5 goal configs have been generated
				if (goal_samples > 5)
				break;
			}
            std::cout<<"AFTER VALIDITY CHECK!!!"<<std::endl;
		}
		// We want to sample stat.-stable configs (not intended to find a grasp_position for right arm)
		else if (double_support_achieved == true && grasp_position_reached == false && sample_goal_configs == false)
		{

			//Assign the robot state to the Kinematic Model
            robot_state::RobotState state(p_s_->getRobotModel());
            state.setToDefaultValues();

            //Set Joint Values
            //state.setStateValues(joint_values);
            state.setVariablePositions(joint_values);

			//Check if the given state is valid. This means checking for collisions and feasibility (setStateFeasibilityPredicate set to isFeasible()-function).
            //if (!p_s_->isStateValid(state))
            //{
            if (isFeasible(state,true))
            {
                std::cout<<" STATE NOT FEASIBLE " << std::endl;
				continue;	//in case the configuration in invalid (self-collision) return and generate a new random config
			}

			else
			{
                std::cout<<" STATE FEASIBLE !!!" << std::endl;

				//Write the configuration into the file
                for (std::size_t i = 0 ; i<joint_names_.size() ; i++)
				{
                    stat_stable_samples_DS_ << joint_values[joint_names_[i]] <<" ";
				}
				//Set cursor to next line (for next configuration)
				stat_stable_samples_DS_ << std::endl;
				//Increment number of statically stable samples found
				stat_stable_samples++;
			}
		}

		else
		{
			//std::cout<<"No valid config found in this iteration" << std::endl;
		}


	} //END of Iterations to sample/collect statically stable double support poses




	//-------- Close the files and plot the number of configurations found
	if (sample_goal_configs == false)
	{
		stat_stable_samples_DS_.close();
		std::cout<<"We found " <<stat_stable_samples<< " statically stable configurations" << std::endl;
		return stat_stable_samples;
	}
	if (sample_goal_configs == true)
	{
		stat_stable_goal_samples_DS_.close();
		std::cout<<"We found " <<goal_samples<< " statically stable GOAL configurations" << std::endl;

		//Sort goal configurations by the ergonomy index
		sort_goal_configs_by_ergonomy(file_destination);

		return goal_samples;
	}



  // we are always successful
  return 0;
}


void StableConfigGenerator::sort_goal_configs_by_ergonomy(char *file_destination = "config.dat")
{
	//+++++++++++++++++++++++++ Read goal config file into array ++++++++++++++++++++++++++++++++++
	//Open the file containing the goal configurations
    std::ifstream config_file(file_destination);

    //Joint Names of the group
    //const std::vector<std::string>& joint_names = p_s_->getRobotModel()->getJointModelGroup(group_name_)->getJointModelNames();


    // --------------- Get the number of configurations in the file --------------------------
	std::string line;
	int num_configs = 0;
	if (config_file.is_open())
	{
		while ( config_file.good() )
		{
		 std::getline (config_file,line);
		 num_configs++;
		}
		//Delete last line containing eof
		num_configs--;
		config_file.close();
	}

	else std::cout << "Unable to open file (sort goal configs)";



	// --------------- Initialize Array storing the values from the file --------------------------
	double **configs;
	configs=new double*[num_configs]; //creates a new array of pointers to double objects
	for(int i=0; i<num_configs; ++i)
    configs[i]=new double[joint_names_.size()+1]; //the joint values + the ergonomy value



	//--------------- Fill the 2D Array --------------------------
	char * pEnd;
	int j = 0;
	char char_line[1000];
	config_file.open(file_destination);
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

         for (unsigned int i = 0 ; i < joint_names_.size() ; ++i)
		 {
		   configs[j][i] = strtod(tmp,&pEnd);
		   //Assign the remaining char elements (pEnd) to the current char pointer
		   tmp = pEnd;
		 }
		 //Last joint value of configuration
         configs[j][joint_names_.size()] = strtod(tmp,NULL);

		 //Next configuration
		 j++;

		}
		config_file.close();
	}

	else std::cout << "Unable to open file (sort goal configs)";


	//+++++++++++++++++++++++++ Sort the configs by their ergonomy index ++++++++++++++++++++++++++++++++++
	//Collect ergonomy values in a vector
	std::vector<double> ergonomy(num_configs);
	for (int i = 0 ; i < num_configs ; i++)
		ergonomy[i] = configs[i][0];


	//Sort ergonomy values by index
	std::vector<int> indices_sorted(num_configs);
	int entry = 0;
	int index_min_element = 0;
	double value_min_element = 0.0;
	for (unsigned int n = 0 ; n < ergonomy.size() ; n++)
	{
		//value_min_element = 10000.0;
		value_min_element = 0.0;
		//Find smallest/best ergonomy value
		for (unsigned int i = 0 ; i < ergonomy.size() ; i++)
		{
//			if (ergonomy[i] < value_min_element)
//			{
//				value_min_element = ergonomy[i];
//				index_min_element = i;
//			}
//			else{}
			if (ergonomy[i] > value_min_element)
			{
				value_min_element = ergonomy[i];
				index_min_element = i;
			}
			else{}
		}

		//std::cout<<"Config index:" <<index_min_element<< std::endl;
		//std::cout<<"Ergonomy value :" <<ergonomy[index_min_element]<< std::endl;

		//Store index of config with the smallest ergonomy value found in the ergonomy value vector
		indices_sorted[entry] = index_min_element;
		entry++;

		//Set value of vector element providing the smallest ergonomy value to a high value to signalize in the next iteration
		//that this element has already been chosen
		//ergonomy[index_min_element] = 1000.0;

		ergonomy[index_min_element] = 0.0;

	}

	//++++++++++++++++++++++ Write the configs in increasing order of their ergonomy index ++++++++++++++++++++++++++++++++++

	stat_stable_goal_samples_DS_.open(file_destination); // opens the file
	if( !stat_stable_goal_samples_DS_ )
	 {
		// file couldn't be opened
		std::cerr << "Error: file could not be opened" << std::endl;
		exit(1);
	 }

	for (int i = 0 ; i < num_configs ; i++)
	{
		//Write the configuration into the file
        for (std::size_t j = 1 ; j< (joint_names_.size()+1) ; j++) //Skip the ergonomy value -> start with first joint value
		{
			stat_stable_goal_samples_DS_ << configs[indices_sorted[i]][j] <<" ";
		}
		//Set cursor to next line (for next configuration)
		stat_stable_goal_samples_DS_ << std::endl;
	}

	//Close the file
	stat_stable_goal_samples_DS_.close();


}



//Function to get the best goal config (index 0)
void StableConfigGenerator::loadGoalConfig(std::vector<double>& config, char *file_destination)
{
	// --------------- Get the number of configurations in the file --------------------------
	std::string line;
	int num_configs = 0;
	std::ifstream config_file(file_destination);
	if (config_file.is_open())
	{
		while ( config_file.good() )
		{
		 std::getline (config_file,line);
		 num_configs++;
		}
		//Delete last line containing eof
		num_configs--;
		std::cout <<"File contains " <<num_configs << " goal configurations" << std::endl;
		config_file.close();
	}

	else std::cout << "Unable to open file (load goal config)";



	/// --------------- Initialize Array storing the values from the file --------------------------
	int num_joints = config.size();
	double **ss_configs; //two * are needed because it is a pointer to a pointer

	ss_configs=new double*[num_configs]; //creates a new array of pointers to double objects
	for(int i=0; i<num_configs; ++i)
	ss_configs[i]=new double[num_joints+1]; //the joint values + the ergonomy value



	//--------------- Fill the 2D Array --------------------------
	char * pEnd;
	int j = 0;
	char char_line[1000];
	config_file.open(file_destination);
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

		 for (int i = 0 ; i < (num_joints-1) ; ++i)
		 {
		   ss_configs[j][i] = strtod(tmp,&pEnd);
		   //Assign the remaining char elements (pEnd) to the current char pointer
		   tmp = pEnd;
		 }
		 //Last joint value of configuration
		 ss_configs[j][num_joints-1] = strtod(tmp,NULL);

		 //Next configuration
		 j++;

		}
		config_file.close();
	}

	else std::cout << "Unable to open file (load goal config)";

	//Select a random configuration from the goal configurations file
	std::srand(std::time(NULL));
	int lb = 0, ub = num_configs - 1;
	int random_config_index = lb + (rand() % ((ub - lb) + 1));


	//Select a specific goal configuration
	random_config_index = 0;

	//Output the selected goal configuration index
	std::cout<< "Goal configuration number: " << random_config_index <<" chosen"<<std::endl;

	for (int i = 0 ; i < num_joints; i++)
	{
	 config[i] = ss_configs[random_config_index][i];
	 //std::cout<< config[i] << " ";
	}
}


//Fix Joint name order from RobotModel to order used by planner
std::vector<std::string> StableConfigGenerator::joint_name_order_RobModel_to_Planner(const std::vector<std::string> j_names)
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


}//END of namespace
