/* Author: Felix Burget */

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <hrl_kinematics/TestStability.h>

#include <fstream>

//Set the file containing configurations
const char* FILENAME = "config_database/ssc_double_support_goal_first.dat";
const char* FILENAME_SOLUTION_PATH = "solutions/solution_path_first.dat";


void setJointValues(int scenario)
{
	ros::NodeHandle nh;
	//Loads urdf, srdf, creates kin.models and planning scene, subscribes to a bunch of ROS topics
	planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
	//Publisher for planning_scene topic
	ros::Publisher pub_scene = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

	ros::Publisher pub_scene_1 = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_1", 1);
	ros::Publisher pub_scene_2 = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_2", 1);
	ros::Publisher pub_scene_3 = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_3", 1);
	ros::Publisher pub_scene_4 = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_4", 1);
	ros::Publisher pub_scene_5 = nh.advertise<moveit_msgs::PlanningScene>("planning_scene_5", 1);

	// Get the planning Scene from PlanningSceneMonitor object psm
	const planning_scene::PlanningScenePtr &scene = psm.getPlanningScene();

	std::string primary_planning_group = "";
	std::string secondary_planning_group = "";

	//Test Stability object (for projected_COM visualization)
	hrl_kinematics::TestStability stability;

	ros::Publisher pcom_pub;
	pcom_pub = nh.advertise<visualization_msgs::Marker>("com_projected", 10);

	ros::Publisher support_polygon_pub;
	support_polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("support_polygon", 10);

	//Planning Scenario 0 -> Set All Joints to a certain value
	if (scenario == 0)
	{
		//set the planning group(s)
		primary_planning_group = "whole_body_rfoot";


//		//--------------------------- Insert Box into Scene -----------------------------------------------
//		scene->setName("nao_with_box");
//		//create and position Box
//		Eigen::Affine3d t;
//		t = Eigen::Translation3d(0.15, -0.17, 0.02);
//		scene->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.02, 1.0, 0.02)), t);
//		//--------------------------------------------------------------------------------------------


		//---------------------------------- RECONFIGURE THE Kin.Model --------------------
		//Get the urdf and srdf model
		const boost::shared_ptr< const urdf::ModelInterface > & urdf_nao = psm.getPlanningScene()->getRobotModel()->getURDF();
		const boost::shared_ptr< const srdf::Model > & srdf_nao = psm.getPlanningScene()->getRobotModel()->getSRDF();
		//Configure the planning scene (set root joint)
		if(!psm.getPlanningScene()->configure(urdf_nao, srdf_nao, "r_sole"))
		{
		std::cout<< "FAILED to configure planning scene!!!" << std::endl;
		}

		//Print the name of the root joint
		ROS_INFO("Root JOINT is %s ",psm.getPlanningScene()->getKinematicModel()->getRootJointName().c_str());
		ROS_INFO("Root LINK is %s ",psm.getPlanningScene()->getKinematicModel()->getRootLink()->getName().c_str());
		//-----------------------------------------------------------------------


		//----------------------------- Set Robot State ------------------------------------------
		//Get current robot state
		robot_state::RobotState &state = scene->getCurrentState();

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getJointModelNames();


		std::vector<double> joint_values(joint_names.size());
		joint_values[0] = -0.0106252;// 0.13026;
		joint_values[1] = -0.608278;//-0.946418;
		joint_values[2] = 1.13076;//1.57326;
		joint_values[3] = -0.463787; //-0.992081;
		joint_values[4] = 0.0455536; //0.149099;
		joint_values[5] = 0.261699; //1.5708;
		joint_values[6] = 0.467684; //0.17;
		joint_values[7] = -0.893101; //0;
		joint_values[8] = -0.603029; //-0.036;
		joint_values[9] = -0.0971596; //0;
		joint_values[10] = 0.17478; //0.462729;
		joint_values[11] = -0.662864; //-0.600352;
		joint_values[12] = -0.436721; //1.49342;
		joint_values[13] = 0.831104; //1.18827;
		joint_values[14] = 0.226537; //-0.753719;
		joint_values[15] = 0; //0;
		joint_values[16] = 0.0474611; //0.149311;
		joint_values[17] = -0.488982; //-1.17933;
		joint_values[18] = 1.17431; //1.92549;
		joint_values[19] = -0.626632; //-1.1114;
		joint_values[20] = -0.0125314; //0.130073;

		//Map containing joint names and values
		std::map<std::string, double> nvalues;

		for (unsigned int i = 0; i != joint_names.size() ; i++)
		{
		 nvalues[joint_names[i]] = joint_values[i];
		}


		//Set current robot state
		state.setStateValues(nvalues);
		//----------------------------------------------------------------------------------------


		//--------------------------- Insert Shelf into Scene -----------------------------------------------
		//NOTE: Position of table is w.r.t right foot frame
		scene->setName("nao_with_shelf");
		//thickness bottom plate
		double thickness_bottom_plate = 0.04;
		double thickness_top_plate = 0.017;
		double thickness_shelfs = 0.014;
		//Shelf edge position
		double shelf_x = 0.30;
		double shelf_y = -0.06;
		double shelf_z = thickness_bottom_plate/2;
		//create bottom plate
		Eigen::Affine3d t;
		t = Eigen::Translation3d(shelf_x, shelf_y, shelf_z);
		scene->getCollisionWorld()->addToObject("plate_bottom", shapes::ShapeConstPtr(new shapes::Box(0.27, 0.31, thickness_bottom_plate)), t);

		//Create left side wall
		t = Eigen::Translation3d(shelf_x + 0.0175, shelf_y + 0.132 , shelf_z + 0.28);
		scene->getCollisionWorld()->addToObject("left_wall", shapes::ShapeConstPtr(new shapes::Box(0.235, 0.014, 0.52)), t);

		//Create right side wall
		t = Eigen::Translation3d(shelf_x + 0.0175, shelf_y - 0.132 , shelf_z + 0.28);
		scene->getCollisionWorld()->addToObject("right_wall", shapes::ShapeConstPtr(new shapes::Box(0.235, 0.014, 0.52)), t);

		//Create top plate
		t = Eigen::Translation3d(shelf_x, shelf_y, shelf_z + 0.5485);
		scene->getCollisionWorld()->addToObject("plate_top", shapes::ShapeConstPtr(new shapes::Box(0.27, 0.31, thickness_top_plate)), t);

		//Create bottom shelf
		t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.147);
		scene->getCollisionWorld()->addToObject("shelf_bottom", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);

		//Create center shelf
		t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.2775);
		scene->getCollisionWorld()->addToObject("shelf_center", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);

		//Create center shelf
		t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.408);
		scene->getCollisionWorld()->addToObject("shelf_center", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);
		//--------------------------------------------------------------------------------------------


	}


	//Planning Scenario 1 -> Avoid Obstacle
	else if (scenario == 1)
	{
		//set the planning group(s)
		primary_planning_group = "right_arm";

		//--------------------------- Insert Box into Scene -----------------------------------------------
		scene->setName("nao_with_box");
		//create and position Box
		Eigen::Affine3d t;
		t = Eigen::Translation3d(0.15, -0.17, 0.02);
		scene->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.02, 1.0, 0.02)), t);
		//--------------------------------------------------------------------------------------------


		//--------------------------- Insert Table into Scene -----------------------------------------------
//		scene->setName("nao_with_box");
//		//create and position Box
//		Eigen::Affine3d t;
//		t = Eigen::Translation3d(0.25, 0.0, 0.0);
//		scene->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.3, 0.01)), t);
		//--------------------------------------------------------------------------------------------

		//----------------------------- Set Robot State ------------------------------------------
		//Get current robot state
		robot_state::RobotState &state = scene->getCurrentState();

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getJointModelNames();

		//
		std::map<std::string, double> nvalues;

//		nvalues[joint_names[0]] = 0.0;
//		nvalues[joint_names[1]] = -0.8;
//		nvalues[joint_names[2]] = 0.1;
//		nvalues[joint_names[3]] = 0.1;
//		nvalues[joint_names[4]] = 0.1;

		//Set current robot state
		state.setStateValues(nvalues);
		//----------------------------------------------------------------------------------------
	}
	//Planning Scenario 2 -> Avoid Self Collision
	else if(scenario == 2)
	{
		//set the planning group(s)
		primary_planning_group = "right_arm";
		secondary_planning_group = "left_arm";
		//----------------------------- Set Robot State ------------------------------------------
		//Get current robot state
		robot_state::RobotState &state = scene->getCurrentState();

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names_pr = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names_pr = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getJointModelNames();

		// Map for joint values
		std::map<std::string, double> nvalues;

		//Joint values for Primary planning group
		nvalues[joint_names_pr[0]] = -0.8;
		nvalues[joint_names_pr[1]] = 0.3;
		nvalues[joint_names_pr[2]] = 0.1;
		nvalues[joint_names_pr[3]] = 0.3;
		nvalues[joint_names_pr[4]] = 0.1;

		//for (int i = 0; i != joint_names.size() ; i++)
		//{
		//nvalues[joint_names[i]] = 0.1;
		//}

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names_sec = scene->getKinematicModel()->getJointModelGroup(secondary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names_sec = scene->getKinematicModel()->getJointModelGroup(secondary_planning_group)->getJointModelNames();

		//Joint values for Primary planning group
		nvalues[joint_names_sec[0]] = -0.4;
		nvalues[joint_names_sec[1]] = -0.4;
		nvalues[joint_names_sec[2]] = 0.0;
		nvalues[joint_names_sec[3]] = -0.9;
		nvalues[joint_names_sec[4]] = 0.1;


		//Set current robot state
		state.setStateValues(nvalues);
		//----------------------------------------------------------------------------------------


	}


	//Planning Scenario 3 -> Arms motion
	else if(scenario == 3)
	{
		//set the planning group(s)
		primary_planning_group = "right_arm";
		secondary_planning_group = "left_arm";
		//----------------------------- Set Robot State ------------------------------------------
		//Get current robot state
		robot_state::RobotState &state = scene->getCurrentState();

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names_pr = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names_pr = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getJointModelNames();

		// Map for joint values
		std::map<std::string, double> nvalues;

		//Joint values for Primary planning group
		nvalues[joint_names_pr[0]] = -0.8;
		nvalues[joint_names_pr[1]] = 0.3;
		nvalues[joint_names_pr[2]] = 0.1;
		nvalues[joint_names_pr[3]] = 0.3;
		nvalues[joint_names_pr[4]] = 0.1;

		//for (int i = 0; i != joint_names.size() ; i++)
		//{
		//nvalues[joint_names[i]] = 0.1;
		//}

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names_sec = scene->getKinematicModel()->getJointModelGroup(secondary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names_sec = scene->getKinematicModel()->getJointModelGroup(secondary_planning_group)->getJointModelNames();

		//Joint values for Primary planning group
		nvalues[joint_names_sec[0]] = 0.4;
		nvalues[joint_names_sec[1]] = 0.4;
		nvalues[joint_names_sec[2]] = 0.0;
		nvalues[joint_names_sec[3]] = -0.9;
		nvalues[joint_names_sec[4]] = 0.1;


		//Set current robot state
		state.setStateValues(nvalues);
		//----------------------------------------------------------------------------------------


	}


	//Planning Scenario 4 -> with root_link = r_sole
	else if(scenario == 4)
	{
		//set the planning group(s)
		primary_planning_group = "whole_body_rfoot";

		//----------------------------- Set Robot State ------------------------------------------
		//Get current robot state
		robot_state::RobotState &state = scene->getCurrentState();

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getJointModelNames();

		// Map for joint values
		std::map<std::string, double> nvalues;

		//Set all joint values to zero
		for (unsigned int i = 0; i != joint_names.size() ; i++)
		{
			std::cout<< joint_names[i] << std::endl;
			nvalues[joint_names[i]] = 0.0;
		}

		//Right Ankle Roll (to move CoM over right foot)
		nvalues["RAnkleRoll"] = -0.2618;


		//Config for left arm
		nvalues["LShoulderPitch"] = 0.4;
		nvalues["LShoulderRoll"] = 0.4;
		nvalues["LElbowYaw"] = 0.0;
		nvalues["LElbowRoll"] = -0.9;
		nvalues["LWristYaw"] = 0.1;

		//Config for right arm
		nvalues["RShoulderPitch"] = -0.8;
		nvalues["RShoulderRoll"] = 0.3;
		nvalues["RElbowYaw"] = 0.1;
		nvalues["RElbowRoll"] = 0.3;
		nvalues["RWristYaw"] = 0.1;


		//assign joint values to state
		state.setStateValues(nvalues);
		//----------------------------------------------------------------------------------------

	}



	//Planning Scenario 5 -> set a configuration
	else if(scenario == 5)
	{
		//set the planning group(s)
		primary_planning_group = "whole_body_rfoot";


		//---------------------------------- RECONFIGURE THE Kin.Model --------------------
		//Get the urdf and srdf model
		const boost::shared_ptr< const urdf::ModelInterface > urdf_nao = psm.getPlanningScene()->getUrdfModel();
		const boost::shared_ptr< const srdf::Model > srdf_nao = psm.getPlanningScene()->getSrdfModel();
		//Configure the planning scene (set root joint)
		if(!psm.getPlanningScene()->configure(urdf_nao, srdf_nao, "r_sole"))
		{
		std::cout<< "FAILED to configure planning scene!!!" << std::endl;
		}

		//Print the name of the root joint
		ROS_INFO("Root JOINT is %s ",psm.getPlanningScene()->getKinematicModel()->getRootJointName().c_str());
		ROS_INFO("Root LINK is %s ",psm.getPlanningScene()->getKinematicModel()->getRootLink()->getName().c_str());
		//---------------------------------------------------------------------------------

		//----------------------------- Set Robot State ------------------------------------------
		//Get current robot state
		robot_state::RobotState &state = scene->getCurrentState();

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getJointModelNames();


		// --------------- Get the number of configurations in the file --------------------------
		std::string line;
		int num_configs = 0;
		std::ifstream config_file(FILENAME);
		if (config_file.is_open())
		{
			while ( config_file.good() )
			{
			 std::getline (config_file,line);
			 num_configs++;
			}
			//Delete last line containing eof
			num_configs--;
			std::cout <<"File contains " <<num_configs << " configurations" << std::endl;
			config_file.close();
		}

		else std::cout << "Unable to open file !!!!" << std::endl;



		/// --------------- Initialize Array storing the values from the file --------------------------
		double **ss_configs; //two * are needed because it is a pointer to a pointer

		int num_joints = joint_names.size();

		ss_configs=new double*[num_configs]; //creates a new array of pointers to double objects
		for(int i=0; i<num_configs; ++i)
		ss_configs[i]=new double[num_joints];



		//--------------- Fill the 2D Array --------------------------
		char * pEnd;
		int j = 0;
		char char_line[1000];
		config_file.open(FILENAME);
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

			 for (unsigned int i = 0 ; i < (joint_names.size()-1) ; ++i)
			 {
			   ss_configs[j][i] = strtod(tmp,&pEnd);
			   //Assign the remaining char elements (pEnd) to the current char pointer
			   tmp = pEnd;
			 }
			 //Last joint value of configuration
			 ss_configs[j][joint_names.size()-1] = strtod(tmp,NULL);

			 //Next configuration
			 j++;

			}
			config_file.close();
		}

		else std::cout << "Unable to open file !!!!" << std::endl;



		// ------------------------ Set a configuration from the set of stat.-stable configurations ------------------------
		// Map for joint values
		std::map<std::string, double> nvalues;

		//Set all joint values to zero
		for (unsigned int i = 0; i != joint_names.size() ; i++)
		{
			//std::cout<< joint_names[i] << std::endl;
			nvalues[joint_names[i]] = 0.0;
		}

		//Show all configurations from the file
		moveit_msgs::PlanningScene mymsg;
		for (int current_config_index = 0; current_config_index < num_configs ; current_config_index++)
		{
			// Press key to show next configuration
			std::string dummy;
			std::getline(std::cin, dummy);

			for (unsigned int i = 0; i < joint_names.size() ; i++)
			{
				//std::cout<< joint_names[i] << std::endl;
				nvalues[joint_names[i]] = ss_configs[current_config_index][i];
			}
			state.setStateValues(nvalues);

			//Send current configuration to planning scene
			psm.getPlanningScene()->getPlanningSceneMsg(mymsg);
			//mymsg.planning_frame = "r_sole";
			pub_scene.publish(mymsg);

			break;

		}

		//Select a certain configuration from the file
//		int config_index = 4;
//		for (unsigned int i = 0; i < joint_names.size() ; i++)
//		{
//			//std::cout<< joint_names[i] << std::endl;
//			nvalues[joint_names[i]] = ss_configs[config_index][i];
//		}
//		state.setStateValues(nvalues);

		// Sequence of Joints (when r_sole is set as root joint)
		//		0.RAnkleRoll
		//		1.RAnklePitch
		//		2.RKneePitch
		//		3.RHipPitch
		//		4.RHipRoll
		//		5.LShoulderPitch
		//		6.LShoulderRoll
		//		7.LElbowYaw
		//	    8.LElbowRoll
		//	    9.LWristYaw
		//	    10.RShoulderPitch
		//	    11.RShoulderRoll
		//		12.RElbowYaw
		//		13.RElbowRoll
		//		14.RWristYaw
		//		15.LHipYawPitch
		//		16.LHipRoll
		//		17.LHipPitch
		//		18.LKneePitch
		//		19.LAnklePitch
		//		20.LAnkleRoll

	}



	//Planning Scenario 6 -> play motion planning trajectory
	else if(scenario == 6)
	{
		//set the planning group(s)
		primary_planning_group = "whole_body_rfoot";


		//--------------------------- Insert Box into Scene -----------------------------------------------
//		psm.getPlanningScene()->setName("nao_with_box");
//		//create and position Box
//		Eigen::Affine3d t;
//		t = Eigen::Translation3d(0.15, -0.17, 0.04);
//		psm.getPlanningScene()->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.015, 1.0, 0.015)), t);
		//--------------------------------------------------------------------------------------------



		//---------------------------------- RECONFIGURE THE Kin.Model --------------------
		//Get the urdf and srdf model
		const boost::shared_ptr< const urdf::ModelInterface > urdf_nao = psm.getPlanningScene()->getUrdfModel();
		const boost::shared_ptr< const srdf::Model > srdf_nao = psm.getPlanningScene()->getSrdfModel();
		//Configure the planning scene (set root joint)
		if(!psm.getPlanningScene()->configure(urdf_nao, srdf_nao, "r_sole"))
		{
		std::cout<< "FAILED to configure planning scene!!!" << std::endl;
		}

		//Print the name of the root joint
		ROS_INFO("Root JOINT is %s ",psm.getPlanningScene()->getKinematicModel()->getRootJointName().c_str());
		ROS_INFO("Root LINK is %s ",psm.getPlanningScene()->getKinematicModel()->getRootLink()->getName().c_str());
		//---------------------------------------------------------------------------------


		//--------------------------- Insert Table into Scene -----------------------------------------------
		//NOTE: Position of table is w.r.t right foot frame
		scene->setName("nao_with_table");
		//create and position Box
		Eigen::Affine3d t;
		t = Eigen::Translation3d(0.32, 0.05, 0.333);
		scene->getCollisionWorld()->addToObject("table", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.3, 0.03)), t);
		//--------------------------------------------------------------------------------------------

		//----------------------------- Set Robot State ------------------------------------------
		//Get current robot state
		robot_state::RobotState &state = scene->getCurrentState();

		//Get the Name of the Links and Joints of a specific group
		std::vector<std::string> link_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getUpdatedLinkModelNames();
		const std::vector<std::string>& joint_names = scene->getKinematicModel()->getJointModelGroup(primary_planning_group)->getJointModelNames();


		// --------------- Get the number of configurations in the file --------------------------
		std::string line;
		int num_configs = 0;
		std::ifstream config_file(FILENAME_SOLUTION_PATH);
		if (config_file.is_open())
		{
			while ( config_file.good() )
			{
			 std::getline (config_file,line);
			 num_configs++;
			}
			//Delete last line containing eof
			num_configs--;
			std::cout <<"File contains " <<num_configs << " configurations" << std::endl;
			config_file.close();
		}

		else std::cout << "Unable to open file !!!!" << std::endl;



		/// --------------- Initialize Array storing the values from the file --------------------------
		double **ss_configs; //two * are needed because it is a pointer to a pointer

		int num_joints = joint_names.size();

		ss_configs=new double*[num_configs]; //creates a new array of pointers to double objects
		for(int i=0; i<num_configs; ++i)
		ss_configs[i]=new double[num_joints];


		//--------------- Fill the 2D Array --------------------------
		char * pEnd;
		int j = 0;
		char char_line[1000];
		config_file.open(FILENAME_SOLUTION_PATH);
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

			 for (unsigned int i = 0 ; i < (joint_names.size()-1) ; ++i)
			 {
			   ss_configs[j][i] = strtod(tmp,&pEnd);
			   //Assign the remaining char elements (pEnd) to the current char pointer
			   tmp = pEnd;
			 }
			 //Last joint value of configuration
			 ss_configs[j][joint_names.size()-1] = strtod(tmp,NULL);

			 //Next configuration
			 j++;

			}
			config_file.close();
		}

		else std::cout << "Unable to open file !!!!" << std::endl;



		// ------------------------ Set a configuration from the set of stat.-stable configurations ------------------------
		// Map for joint values
		std::map<std::string, double> nvalues;

		//Set all joint values to zero
		for (unsigned int i = 0; i != joint_names.size() ; i++)
		{
			//std::cout<< joint_names[i] << std::endl;
			nvalues[joint_names[i]] = 0.0;
		}
		//Head joint angles required by stability node (to compute projected_CoM)
		nvalues["HeadPitch"] = 0.0;
		nvalues["HeadYaw"] = 0.0;
		nvalues["RHipYawPitch"] = 0.0;

		//Show all configurations from the file
		int current_planning_scene = 0;
		for (int current_config_index = 0; current_config_index < num_configs ; current_config_index++)
		{
			// Press key to show next configuration
			std::string dummy;
			std::getline(std::cin, dummy);

			for (unsigned int i = 0; i < joint_names.size() ; i++)
			{

				//std::cout<< joint_names[i] << std::endl;
				nvalues[joint_names[i]] = ss_configs[current_config_index][i];
			}
			state.setStateValues(nvalues);


			moveit_msgs::PlanningScene mymsg;
			//Send current configuration to planning scene
			psm.getPlanningScene()->getPlanningSceneMsg(mymsg);
			mymsg.robot_model_root = "base_link";
			//mymsg.robot_model_root = "r_sole";
			if (current_planning_scene == 0)
			pub_scene.publish(mymsg);
//			if (current_planning_scene == 80)
//			pub_scene_1.publish(mymsg);
//			if (current_planning_scene == 170)
//			pub_scene_2.publish(mymsg);
//			if (current_planning_scene == 310)
//			pub_scene_3.publish(mymsg);
//			if (current_planning_scene == 420)
//			pub_scene_4.publish(mymsg);
//			if (current_planning_scene == 504)
//			pub_scene_5.publish(mymsg);

			//current_planning_scene++;
			//std::cout<<current_planning_scene<<std::endl;



			bool statically_stable = false;
			statically_stable = stability.isPoseStable(nvalues,hrl_kinematics::TestStability::SUPPORT_DOUBLE);

			// print info
			 tf::Point com = stability.getCOM();

			 visualization_msgs::Marker com_marker;
			 com_marker = stability.getCOMMarker();
			 com_marker.header.frame_id = "base_link";

			 //Publish CoM
			 pcom_pub.publish(com_marker);

			 //Publish Support Polygon
			 support_polygon_pub.publish(stability.getSupportPolygon());



		}



		// Sequence of Joints (when r_sole is set as root joint)
		//		0.RAnkleRoll
		//		1.RAnklePitch
		//		2.RKneePitch
		//		3.RHipPitch
		//		4.RHipRoll
		//		5.LShoulderPitch
		//		6.LShoulderRoll
		//		7.LElbowYaw
		//	    8.LElbowRoll
		//	    9.LWristYaw
		//	    10.RShoulderPitch
		//	    11.RShoulderRoll
		//		12.RElbowYaw
		//		13.RElbowRoll
		//		14.RWristYaw
		//		15.LHipYawPitch
		//		16.LHipRoll
		//		17.LHipPitch
		//		18.LKneePitch
		//		19.LAnklePitch
		//		20.LAnkleRoll

	}




	else
	{std::cout << "No valid Scenario chosen !!!" << std::endl;}

	ros::Duration(0.5).sleep();


	moveit_msgs::PlanningScene psmsg;
	psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
	//psmsg.planning_frame = "r_sole";
	pub_scene.publish(psmsg);
	sleep(1);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "initScene");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    int scenario = 1;
    if(argc > 1) {
    std::stringstream s(argv[1]);
    s >> scenario;
    }
    setJointValues(scenario);

    ros::shutdown();
    return 0;
}
 
