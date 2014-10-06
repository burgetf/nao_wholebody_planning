/*
 * planning_example.cpp
 *
 *  Created on: Aug 17, 2012
 *      Author: Felix Burget
 */

#include <ros/ros.h>
//#include <planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/DisplayTrajectory.h>

//Include PLanner messages
#include <rrt_planner_msgs/Generate_DS_Configs.h>
#include <rrt_planner_msgs/Compute_Goal_Config.h>
#include <rrt_planner_msgs/Compute_Motion_Plan.h>
#include <rrt_planner_msgs/Compute_Linear_Manipulation_Plan.h>
#include <rrt_planner_msgs/Compute_Circular_Manipulation_Plan.h>


//Include Actionlib and messages
#include <nao_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>





//Fix Joint name order from RobotModel to order used by planner
std::vector<std::string> joint_name_order_RobModel_to_Planner(const std::vector<std::string> j_names)
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



//void setState_in_PlanningScene( robot_state::RobotState state, planning_scene_monitor::PlanningSceneMonitor psm)
//{
//    planning_scene_monitor::LockedPlanningSceneRW p_s_w(psm);
//    p_s_w->setCurrentState(state);
//}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_example_simulation");

  ros::NodeHandle nh;

  //Index of service to call/test
  int ACTIVE_SERVICE = 2;
  if(argc > 1) {
  std::stringstream s(argv[1]);
  s >> ACTIVE_SERVICE;
  }

  //---------------------------------------------- SETUP -----------------------------------------
  //Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitor psm("robot_description");
  //boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> psm =  boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor>(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
  //planning_scene_monitor::LockedPlanningSceneRO p_s(psm);



  //Planning Scene Publisher
  ros::Publisher scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Duration(0.5).sleep();

  //Trajectory publisher
  ros::Publisher pub_traj= nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);
   


  //Set the planning scene name (not used anywhere else so far)
  //psm.getPlanningScene()->setName("nao_wbmp");



  //---------------------------------- SOME TESTING --------------------
  std::cout<<"getRootJointName: "<<psm.getPlanningScene()->getRobotModel()->getRootJointName() <<std::endl;
  std::cout<<"getRootLinkName: "<<psm.getPlanningScene()->getRobotModel()->getRootLinkName() <<std::endl;
  std::cout<<"getPlanningFrame: "<<psm.getPlanningScene()->getPlanningFrame() <<std::endl;
  std::cout<<"getModelFrame: "<<psm.getPlanningScene()->getRobotModel()->getModelFrame() <<std::endl<<std::endl;

//  const std::vector<std::string> &link_names = psm.getPlanningScene()->getRobotModel()->getLinkModelNames();
//  //Set joint values/names of the map
//  for (unsigned int i = 0; i != link_names.size() ; i++)
//  {
//    std::cout<<link_names[i] <<std::endl;
//  }

 //psm.getPlanningScene()->getRobotModel()->getURDF();
 //psm.getPlanningScene()->getRobotModel()->getSRDF();

  const boost::shared_ptr< const srdf::Model > &srdf_nao = psm.getPlanningScene()->getRobotModel()->getSRDF();

  std::cout<<"srdf getName: "<<srdf_nao->getName() <<std::endl;

  const std::vector<srdf::Model::VirtualJoint> &vjoints = srdf_nao->getVirtualJoints();
  for (std::size_t i = 0 ; i < vjoints.size() ; ++i)
  {
      std::cout<<"vjoint Type: "<< vjoints[i].type_ <<std::endl;
      std::cout<<"vjoint Name: "<< vjoints[i].name_<<std::endl;
      std::cout<<"vjoint parent_frame_: "<< vjoints[i].parent_frame_<<std::endl;
      std::cout<<"vjoint child_link_: "<< vjoints[i].child_link_<<std::endl<<std::endl;
  }



  //------------------------------------------------------



//  //---------------------------------- RECONFIGURE THE Kin.Model --------------------
//  //Get the urdf and srdf model
//  const boost::shared_ptr< const urdf::ModelInterface > urdf_nao = psm.getPlanningScene()->getUrdfModel();
//  const boost::shared_ptr< const srdf::Model > srdf_nao = psm.getPlanningScene()->getSrdfModel();
//  //Configure the planning scene (set root joint)
//  if(!psm.getPlanningScene()->configure(urdf_nao, srdf_nao, "r_sole"))
//  {
//  std::cout<< "FAILED to configure planning scene!!!" << std::endl;
//  }

//  //Print the name of the root joint
//  ROS_INFO("Root JOINT is %s ",psm.getPlanningScene()->getKinematicModel()->getRootJointName().c_str());
//  ROS_INFO("Root LINK is %s ",psm.getPlanningScene()->getKinematicModel()->getRootLink()->getName().c_str());
//  //-----------------------------------------------------------------------


   //--------------------------- Set initial config and publish it onto planning scene ----------
    //planning_models::KinematicState &state = psm.getPlanningScene()->getCurrentState();
    robot_state::RobotState state(psm.getPlanningScene()->getRobotModel());
    state.setToDefaultValues();
    //robot_state::RobotState &state = psm.getPlanningScene()->getCurrentState();

    //Get the Name of the Links and Joints of a specific group
    //std::vector<std::string> link_names = psm.getPlanningScene()->getKinematicModel()->getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelNames();
    //const std::vector<std::string>& joint_names = psm.getPlanningScene()->getKinematicModel()->getJointModelGroup("whole_body_rfoot")->getJointModelNames();
    const std::vector<std::string>& joint_names = psm.getPlanningScene()->getRobotModel()->getJointModelGroup("whole_body_rfoot")->getJointModelNames();


    std::vector<std::string> joint_names_ = joint_name_order_RobModel_to_Planner(joint_names);


    //Define init poses
    std::vector<double> init_pose(21);
    //Default values
    init_pose[0] = 0.00000;  //RAnkleRoll
    init_pose[1] = -0.349066;  //RAnklePitch
    init_pose[2] = 0.698132;  //RKneePitch
    init_pose[3] = -0.436332;  //RHipPitch
    init_pose[4] = 0.00000;  //RHipRoll
    init_pose[5] = 1.39626;  //LShoulderPitch
    init_pose[6] = 0.349066;  //LShoulderRoll
    init_pose[7] = -1.39626;  //LElbowYaw
    init_pose[8] = -0.5;//-1.04720;  //LElbowRoll
    init_pose[9] = 0.00000;  //LWristYaw
    init_pose[10] = 1.39626; //RShoulderPitch
    init_pose[11] = -0.349066; //RShoulderRoll
    init_pose[12] = 1.39626; //RElbowYaw
    init_pose[13] = 0.5;//1.04720; //RElbowRoll
    init_pose[14] = 0.00000; //RWristYaw
    init_pose[15] = 0.00000; //LHipYawPitch
    init_pose[16] = 0.00000; //LHipRoll
    init_pose[17] = -0.436332; //LHipPitch
    init_pose[18] = 0.698132; //LKneePitch
    init_pose[19] = -0.349066; //LAnklePitch
    init_pose[20] = 0.00000; //LAnkleRoll
  
    
    //Map containing joint names and values
    std::map<std::string, double> nvalues;
      
   //--------------------------- Drive Robot to initial configuration ---------------------
   if (ACTIVE_SERVICE != 0 && ACTIVE_SERVICE != 1)
   {  
      
      //Drawer Experiment
      if (ACTIVE_SERVICE == 3 || ACTIVE_SERVICE == 2)
      {
        init_pose[0] = 0.00000;  //RAnkleRoll
        init_pose[1] = 0.0;  //RAnklePitch
        init_pose[2] = 0.0;  //RKneePitch
        init_pose[3] = 0.0;  //RHipPitch
        init_pose[4] = 0.00000;  //RHipRoll
        init_pose[5] = 0.0;  //LShoulderPitch
        init_pose[6] = 0.0;  //LShoulderRoll
        init_pose[7] = 0.0;  //LElbowYaw
        init_pose[8] = -0.5;//-1.04720;  //LElbowRoll
        init_pose[9] = 0.00000;  //LWristYaw
        init_pose[10] = 0.0; //RShoulderPitch
        init_pose[11] = 0.0; //RShoulderRoll
        init_pose[12] = 0.0; //RElbowYaw
        init_pose[13] = 0.5;//1.04720; //RElbowRoll
        init_pose[14] = 0.00000; //RWristYaw
        init_pose[15] = 0.00000; //LHipYawPitch
        init_pose[16] = 0.00000; //LHipRoll
        init_pose[17] = 0.0; //LHipPitch
        init_pose[18] = 0.0; //LKneePitch
        init_pose[19] = 0.0; //LAnklePitch
        init_pose[20] = 0.00000; //LAnkleRoll


      }
	
      //Door Experiment
      if (ACTIVE_SERVICE == 4 || ACTIVE_SERVICE == 5)
      {
	
        init_pose[0] = 0.00000;  //RAnkleRoll
        init_pose[1] = -0.349066;  //RAnklePitch
        init_pose[2] = 0.698132;  //RKneePitch
        init_pose[3] = -0.436332;  //RHipPitch
        init_pose[4] = 0.00000;  //RHipRoll
        init_pose[5] = 1.39626;  //LShoulderPitch
        init_pose[6] = 0.349066;  //LShoulderRoll
        init_pose[7] = -1.39626;  //LElbowYaw
        init_pose[8] = -0.5;//-1.04720;  //LElbowRoll
        init_pose[9] = 0.00000;  //LWristYaw
        init_pose[10] = 1.39626; //RShoulderPitch
        init_pose[11] = -0.349066; //RShoulderRoll
        init_pose[12] = 1.39626; //RElbowYaw
        init_pose[13] = 0.5;//1.04720; //RElbowRoll
        init_pose[14] = 0.00000; //RWristYaw
        init_pose[15] = 0.00000; //LHipYawPitch
        init_pose[16] = 0.00000; //LHipRoll
        init_pose[17] = -0.436332; //LHipPitch
        init_pose[18] = 0.698132; //LKneePitch
        init_pose[19] = -0.349066; //LAnklePitch
        init_pose[20] = 0.00000; //LAnkleRoll
	

      }   
      
      
      //Set joint values/names of the map
      for (unsigned int i = 0; i != joint_names_.size() ; i++)
      {
        nvalues[joint_names_[i]] = init_pose[i];
        std::cout<<joint_names_[i] <<std::endl;
      }

      //Set current robot state
      //state.setStateValues(nvalues);
      state.setVariablePositions(nvalues);

      //Apply robot state to planning scene
      //setState_in_PlanningScene(state,psm);
      //planning_scene_monitor::LockedPlanningSceneRW p_s_w(psm);
      psm.getPlanningScene()->setCurrentState(state);
      //delete[] p_s_w;


      //Experiment with collision avoidance
      if (ACTIVE_SERVICE == 5)
      {
	//Insert Collision Objects 
	//--------------------------- Insert Table into Scene -----------------------------------------------
    /*p_s->setName("nao_with_table");
	//create and position Box
	Eigen::Affine3d t;
	t = Eigen::Translation3d(0.32, 0.0, 0.0);
    p_s->getCollisionWorld()->addToObject("table", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.3, 0.03)), t);
	//---------------------------------------------------------------------------------------
	*/
	
	//--------------------------- Insert Shelf into Scene -----------------------------------------------
	//NOTE: Position of shelf is w.r.t right foot frame
    //psm.getPlanningScene()->setName("nao_with_shelf");
	//thickness bottom plate
	double thickness_bottom_plate = 0.04;
	double thickness_top_plate = 0.017;
	double thickness_shelfs = 0.014;
	//Shelf edge position
	double shelf_x = 0.33;
	double shelf_y = -0.06;
	double shelf_z = thickness_bottom_plate/2;
	//create bottom plate
//	Eigen::Affine3d t;
//	t = Eigen::Translation3d(shelf_x, shelf_y, shelf_z);
    //p_s->getCollisionWorld()->addToObject("plate_bottom", shapes::ShapeConstPtr(new shapes::Box(0.27, 0.31, thickness_bottom_plate)), t);

	//Create left side wall
//	t = Eigen::Translation3d(shelf_x + 0.0175, shelf_y + 0.132 , shelf_z + 0.28);
//	p_s->getCollisionWorld()->addToObject("left_wall", shapes::ShapeConstPtr(new shapes::Box(0.235, 0.014, 0.52)), t);

	//Create right side wall
//	t = Eigen::Translation3d(shelf_x + 0.0175, shelf_y - 0.132 , shelf_z + 0.28);
//	p_s->getCollisionWorld()->addToObject("right_wall", shapes::ShapeConstPtr(new shapes::Box(0.235, 0.014, 0.52)), t);

	//Create top plate
//	t = Eigen::Translation3d(shelf_x, shelf_y, shelf_z + 0.5485);
//	p_s->getCollisionWorld()->addToObject("plate_top", shapes::ShapeConstPtr(new shapes::Box(0.27, 0.31, thickness_top_plate)), t);

	//Create bottom shelf
//	t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.147);
//	p_s->getCollisionWorld()->addToObject("shelf_bottom", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);

	//Create center shelf
//	t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.2775);
//	p_s->getCollisionWorld()->addToObject("shelf_center", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);

	//Create center shelf
//	t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.408);
//	p_s->getCollisionWorld()->addToObject("shelf_center", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);
	//--------------------------------------------------------------------------------------------
	
      }
      
      //Publish state on planning scene
      moveit_msgs::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      //psmsg.robot_model_root = "r_sole";
      scene_pub.publish(psmsg);
      sleep(1);
   }


   //--------------------------- Test: GENERATE DS DATABASE Service -----------------------
   if (ACTIVE_SERVICE == 0)
   {
     //Service Client
      ros::ServiceClient ssc_generator = nh.serviceClient<rrt_planner_msgs::Generate_DS_Configs>("generate_ds_database");
      //Service request and response
      rrt_planner_msgs::Generate_DS_Configs srv_ds_configs;
      //Fill request with data
      srv_ds_configs.request.max_samples = 5000;
      srv_ds_configs.request.max_ik_iterations = 5;
      
     if (ssc_generator.call(srv_ds_configs))
      {
          std::cout<<"Service performed!"<<std::endl;
          std::cout<<"A set of "<<srv_ds_configs.response.num_configs_generated<<" statically stable poses has been generated"<<std::endl;
      }
      
      else
      {
        std::cout<<"Failed to call GENERATE DS DATABASE service!"<<std::endl;
        return 1;
      }
      
   }
   
   
   //--------------------------- Test: COMPUTE GOAL CONFIG Service -----------------------
    if (ACTIVE_SERVICE == 1)
   {
     //Service Client
      ros::ServiceClient gc_generator = nh.serviceClient<rrt_planner_msgs::Compute_Goal_Config>("compute_goal_config");
      //Service request and response
      rrt_planner_msgs::Compute_Goal_Config srv_goal_config;
      //Fill request with data
      srv_goal_config.request.right_hand_position.x = 0.2;
      srv_goal_config.request.right_hand_position.y = -0.065;
      srv_goal_config.request.right_hand_position.z = 0.2;
      srv_goal_config.request.right_hand_direction.x = 0.0;
      srv_goal_config.request.right_hand_direction.y = -1.0;
      srv_goal_config.request.right_hand_direction.z = 0.0;
      
     if (gc_generator.call(srv_goal_config))
      {
	  std::cout<<"Service performed!"<<std::endl;
	  
	  //Output the joint values of the goal config generated
	  std::cout<<"Goal configuration generated: "<<std::endl;
	  for (unsigned int i = 0; i <srv_goal_config.response.wb_goal_config.size() ; i++)
	  {
         std::cout<<joint_names_[i]<<": "<<srv_goal_config.response.wb_goal_config[i]<<std::endl;
	  }
      }
      
      else
      {
	std::cout<<"Failed to call COMPUTE GOAL CONFIG service!"<<std::endl;
	return 1;
      }
     
   }
   
   
   //--------------------------- Test: MOTION PLAN Service -----------------------
    
   if (ACTIVE_SERVICE == 2)
   {
      //Service Client
      ros::ServiceClient motion_plan_client = nh.serviceClient<rrt_planner_msgs::Compute_Motion_Plan>("compute_motion_plan");
      //Service request and response
      rrt_planner_msgs::Compute_Motion_Plan srv;
      //Fill request with data
      srv.request.right_hand_position.x = 0.24;
      srv.request.right_hand_position.y = -0.065;
      srv.request.right_hand_position.z = 0.3;
      srv.request.right_hand_direction.x = 0.0;
      srv.request.right_hand_direction.y = -1.0;
      srv.request.right_hand_direction.z = 0.0;

      //Set Start Configuration
      srv.request.wb_start_config = init_pose;

      //Prepare trajectory for simulation
      srv.request.execute_trajectory = false;

      //Call service 
      if (motion_plan_client.call(srv))
      {
          std::cout<<"Service performed!"<<std::endl;
          if(srv.response.success == true)
          {
            //Publish motion plan trajectory
            pub_traj.publish(srv.response.motion_plan);
            std::cout<< "Solution Trajectory published !!!"<< std::endl;
            sleep(1);

	  }
	  else
	  {
	    std::cout<< "Motion Planning failed !!!"<< std::endl;
	  }

	  //std::cout<<srv.response.wb_goal_config.size()<<std::endl;
	  //for (int i = 0; i <srv.response.wb_goal_config.size() ; i++)
	  //   {
	  //     std::cout<<srv.response.wb_goal_config[i]<<std::endl;
	  //   }
	  //std::cout<<"Trajectory points: "<<srv.response.motion_plan.trajectory.joint_trajectory.points.size()<<std::endl;

      }
      else
      {
	std::cout<<"Failed to call MOTION PLAN service!"<<std::endl;
	return 1;
      }
   }
   
    
    
   //--------------------------- Test: LINEAR MANIPULATION PLAN Service -----------------------
   if (ACTIVE_SERVICE == 3)
   {
      //Service Client
      ros::ServiceClient linear_manipulation_plan_client = nh.serviceClient<rrt_planner_msgs::Compute_Linear_Manipulation_Plan>("compute_linear_manipulation_plan");
      //Service request and response
      rrt_planner_msgs::Compute_Linear_Manipulation_Plan srv_manip;
      //Fill request with data
      srv_manip.request.right_hand_position.x = 0.24;
      srv_manip.request.right_hand_position.y = -0.065;
      srv_manip.request.right_hand_position.z = 0.30;
      srv_manip.request.right_hand_direction.x = 0.0;
      srv_manip.request.right_hand_direction.y = -1.0;
      srv_manip.request.right_hand_direction.z = 0.0;
      //Object data
      srv_manip.request.relative_angle = 0.0; //in degrees
      srv_manip.request.object_translation_length = 0.10; //in [m]

      //Set Start Configuration
      srv_manip.request.wb_start_config = init_pose;

      //Prepare trajectory for simulation
      srv_manip.request.execute_trajectory = false;
      
      //Call service
      if (linear_manipulation_plan_client.call(srv_manip))
      {
	  std::cout<<"Service performed!"<<std::endl;      
	  if(srv_manip.response.success_approach == true)
	  {  
	    //Publish motion plan trajectories
	    pub_traj.publish(srv_manip.response.motion_plan_approach);
	    std::cout<< "Approach Trajectory published !!!"<< std::endl;
	    sleep(1);
	  }
	  else std::cout<< "Motion Planning Approach failed !!!"<< std::endl;
	  
	  if(srv_manip.response.success_manipulation == true)
	  {
	      pub_traj.publish(srv_manip.response.motion_plan_manipulation);
	      std::cout<< "Manipulation Trajectory published !!!"<< std::endl;
	      sleep(1);
	  }  
	  else std::cout<< "Motion Planning Manipulation failed !!!"<< std::endl;
	  
      }
      else
      {
	std::cout<<"Failed to call LINEAR MANIPULATION PLAN service!"<<std::endl;
	return 1;
      }
   }

   
   
   //--------------------------- Test: CIRCULAR MANIPULATION PLAN Service -----------------------
   if (ACTIVE_SERVICE == 4)
   {
      //Service Client
      ros::ServiceClient circular_manipulation_plan_client = nh.serviceClient<rrt_planner_msgs::Compute_Circular_Manipulation_Plan>("compute_circular_manipulation_plan");
      //Service request and response
      rrt_planner_msgs::Compute_Circular_Manipulation_Plan srv_manip;
      //Fill request with data
      srv_manip.request.right_hand_position.x = 0.27;
      srv_manip.request.right_hand_position.y = 0.02;
      srv_manip.request.right_hand_position.z = 0.35;
      srv_manip.request.right_hand_direction.x = 0.0;
      srv_manip.request.right_hand_direction.y = 0.0;
      srv_manip.request.right_hand_direction.z = -1.0;
      //Object data
      srv_manip.request.relative_angle = 0.0; //in degrees
      srv_manip.request.rotation_radius = 0.25; //in [m] e.g. door_width
      srv_manip.request.rotation_angle = 40.0; //in degrees , i.e rotation angle around the hinge
      srv_manip.request.clearance_handle = 0.05; // distance between door and handle [5cm]

      //Set Start Configuration
      srv_manip.request.wb_start_config = init_pose;

      //Prepare trajectory for simulation
      srv_manip.request.execute_trajectory = false;
      
      //Call service
      if (circular_manipulation_plan_client.call(srv_manip))
      {
	  std::cout<<"Service performed!"<<std::endl;
	  //Approach
	  if(srv_manip.response.success_approach == true)
	  {
	    //Publish motion plan trajectories
	    pub_traj.publish(srv_manip.response.motion_plan_approach);
	    std::cout<< "Approach Trajectory published !!!"<< std::endl;
	    sleep(1);
	  }
	  else std::cout<< "Motion Planning Approach failed !!!"<< std::endl;
	  
	  //Manipulation
	  if(srv_manip.response.success_manipulation == true)
	  {
	    pub_traj.publish(srv_manip.response.motion_plan_manipulation);
	    std::cout<< "Manipulation Trajectory published !!!"<< std::endl;
	    sleep(1);
	  } 
	  else std::cout<< "Motion Planning Manipulation failed !!!"<< std::endl;
	  
      }
      else
      {
	std::cout<<"Failed to call CIRCULAR MANIPULATION PLAN service!"<<std::endl;
	return 1;
      }
   }
   
   
   
   
   //++++++++++++++++++++++++++++++++ E X P E R I M E N T S ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
   
  
   //--------------------------- SHELF EXPERIMENT -----------------------
   if (ACTIVE_SERVICE == 5)
   { 
     
      //Service Client
      ros::ServiceClient motion_plan_client = nh.serviceClient<rrt_planner_msgs::Compute_Motion_Plan>("compute_motion_plan");
      //Service request and response
      rrt_planner_msgs::Compute_Motion_Plan srv;
     
   
      //-----Put Hand into upper shelf
      //Fill request with data
      srv.request.right_hand_position.x = 0.27;
      srv.request.right_hand_position.y = -0.06;
      srv.request.right_hand_position.z = 0.36;
      srv.request.right_hand_direction.x = 0.0;
      srv.request.right_hand_direction.y = -1.0;
      srv.request.right_hand_direction.z = 0.0;  
      
      //Prepare trajectory for execution
      srv.request.execute_trajectory = false;
      
      
       //Call service 
      if (motion_plan_client.call(srv))
      {
	   std::cout<<"Service performed!"<<std::endl;
	  if(srv.response.success == true)
	  {
	     std::cout<<"Upper shelf reached"<<std::endl;
	     
	    //Time required for search
	    std::cout<<srv.response.search_time<<std::endl;
	    
	    //Publish motion plan trajectory
	    pub_traj.publish(srv.response.motion_plan);
	    std::cout<< "Solution Trajectory published !!!"<< std::endl;
	    sleep(1);
	    
	  }
	  else
	  {
	    std::cout<< "Motion Planning failed !!!"<< std::endl;
	  }
      }
      
      else
      {
        std::cout<<"Failed to call MOTION PLAN service!"<<std::endl;
        return 1;
      }
      
      
      //----------------------- Update Planning Scene for next motion plan -------------------
      //Map containing joint names and values
      std::map<std::string, double> new_values;


       //Set joint values/names of the map
      //int last_element_index = srv.response.motion_plan.trajectory.joint_trajectory.points.size()-1;
      //robot_trajectory::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);


      //Get Robot Trajectory from response message
      moveit_msgs::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);
      //Get corresponding joint_trajectory
      trajectory_msgs::JointTrajectory joint_traj = rob_traj.joint_trajectory;
      //Get last element of trajectory
      int last_element_index = joint_traj.points.size()-1;

      for (unsigned int i = 0; i != joint_names_.size() ; i++)
      {

         // rob_traj.joint_trajectory.points[0].
        //robot_state::RobotState rob_state= rob_traj.getLastWayPoint();
        //new_values[joint_names_[i]] = rob_state.getVariablePosition(i);
        new_values[joint_names_[i]] = joint_traj.points[last_element_index].positions[i];
        //std::cout<<joint_names_[i] <<std::endl;
      }
      //Set current robot state
      //state.setStateValues(new_values);
      state.setVariablePositions(new_values);

      //Apply robot state to planning scene
      //setState_in_PlanningScene(state,psm);
      //planning_scene_monitor::LockedPlanningSceneRW p_s_w(psm);
      psm.getPlanningScene()->setCurrentState(state);
      //delete[] p_s_w;
      
       //Publish state on planning scene
      moveit_msgs::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      //psmsg.robot_model_root = "r_sole";
      scene_pub.publish(psmsg);
      sleep(1);
      
      //----------------------------------------------------
     
     
      //Request for second motion plan
      rrt_planner_msgs::Compute_Motion_Plan srv_2;
      
      //-----Put Hand into lower shelf
      srv_2.request.right_hand_position.x = 0.25;
      srv_2.request.right_hand_position.y = -0.08;
      srv_2.request.right_hand_position.z = 0.22;
      srv_2.request.right_hand_direction.x = 0.0;
      srv_2.request.right_hand_direction.y = -1.0;
      srv_2.request.right_hand_direction.z = 0.0;  
      
      //Prepare trajectory for execution
      srv_2.request.execute_trajectory = false;
      
      
         //Call service 
      if (motion_plan_client.call(srv_2))
      {
	   std::cout<<"Service performed!"<<std::endl;
	   
	  
	  if(srv_2.response.success == true)
	  {
	     std::cout<<"Lower shelf reached"<<std::endl;
	     
	     
	     //Time required for search
	     std::cout<<srv_2.response.search_time<<std::endl;
	    
         //for (unsigned int i = 0; i < joint_names_.size() ; i++)
	     // std::cout<<srv_2.response.motion_plan.trajectory.joint_trajectory.points[0].positions[i]<<std::endl;
	     
	    //Publish motion plan trajectory
	    pub_traj.publish(srv_2.response.motion_plan);
	    std::cout<< "Solution Trajectory published !!!"<< std::endl;
	    sleep(1);
	    
	  }
	  else
	  {
	    std::cout<< "Motion Planning failed !!!"<< std::endl;
	  }
      }
      
      else
      {
	std::cout<<"Failed to call MOTION PLAN service!"<<std::endl;
	return 1;
      }
      
   }
   
   
   
    //++++++++++++++++++++++++++++++++ E V A L U A T I O N ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   if (ACTIVE_SERVICE == 6)
   { 
     //Open a file to store the experiment data
     std::ofstream evaluation_data;
     evaluation_data.open("experiment_results/data.dat"); // opens the file
     if( !evaluation_data )
      {
	    // file couldn't be opened
	    std::cerr << "Error: file could not be opened" << std::endl;
	    exit(1);
      }
	 
	 
     //Number of motion plans computed
     int planning_iterations = 10;
     
     //Center point of grasp points window in [m]
     double window_center_x = 0.12;
     double window_center_y = -0.1;
     double window_center_z = 0.2;
     
     //Define window size in x,y plane ()
     std::vector<double> window_size_x(2); 
     window_size_x[0] =  0.2;//0.2; // = window width / 2 in the positive x-axis direction
     window_size_x[1] =  -0.2;//-0.2; // = window width / 2 in the negative x-axis direction
     std::vector<double> window_size_y(2); 
     window_size_y[0] =  0.2;//0.2; // = window width / 2 in the positive y-axis direction
     window_size_y[1] =  -0.2;//-0.2; // = window width / 2 in the negative y-axis direction
     
      //Service Client
      ros::ServiceClient motion_plan_client = nh.serviceClient<rrt_planner_msgs::Compute_Motion_Plan>("compute_motion_plan");
      //Service request and response
      rrt_planner_msgs::Compute_Motion_Plan srv;
      
      /*
      //Init see for random numbers
      srand ( time(NULL) ); 
      
      for (int motion_plan_num = 0 ; motion_plan_num < planning_iterations ; motion_plan_num++)
      {
	 
	  //Get random offset in x-direction
	  int a =  window_size_x[0] *100;
	  int b =  fabs(window_size_x[1]) *100;
	  double tmp = 0, offset_x = 0;
	  tmp = rand() % (a+b) - a; 
	  offset_x = tmp / 100; //Reduces this number to the range you want.
	  
	  //Get random offset in x-direction
	  a =  window_size_y[0] *100;
	  b =  fabs(window_size_y[1]) *100;
	  tmp = 0;
	  double offset_y = 0;
	  tmp = rand() % (a+b) - a; 
	  offset_y = tmp / 100; //Reduces this number to the range you want.
     
          //std::cout<<offset_y<<std::endl;
	  
	  //------- Fill request with data
	  srv.request.right_hand_position.x = window_center_x + offset_x;
	  srv.request.right_hand_position.y = window_center_y + offset_y;
	  srv.request.right_hand_position.z = window_center_z;
	  srv.request.right_hand_direction.x = 0.0;
	  srv.request.right_hand_direction.y = -1.0;
	  srv.request.right_hand_direction.z = 0.0;
	  //Prepare trajectory for simulation
	  srv.request.execute_trajectory = false;
	    
	  */
	 
      //Write data to file 
      evaluation_data<<"motion_plan_num"<<"  "<<"x"<<"  "<<"y"<<"  "<<"z"<<"  "<<"success"<<"  "<<"num_nodes_total"<<"  "<<"planning_time"<<"  "<<"time_generate_goal_config"<< std::endl;
		  
		
      int motion_plan_num = 0;
      for (double current_x = (window_center_x+window_size_x[1]) ; current_x <= (window_center_x+window_size_x[0]) ; current_x = current_x + 0.01)
      {
	  for (double current_y = (window_center_y+window_size_y[1]) ; current_y <= (window_center_y+window_size_y[0]) ; current_y = current_y + 0.01)
	  {
	   
	    //------- Fill request with data
	    srv.request.right_hand_position.x = current_x;
	    srv.request.right_hand_position.y = current_y;
	    srv.request.right_hand_position.z = window_center_z;
	    srv.request.right_hand_direction.x = 0.0;
	    srv.request.right_hand_direction.y = -1.0;
	    srv.request.right_hand_direction.z = 0.0;
	    //Prepare trajectory for simulation
	    srv.request.execute_trajectory = false;

	    //Call service 
	    if (motion_plan_client.call(srv))
	    {
		//std::cout<<"Service performed!"<<std::endl;
		std::cout<< "Motion Plan Number: "<<motion_plan_num;
		std::cout<<" Hand pose: "<<srv.request.right_hand_position.x<<" "<< srv.request.right_hand_position.y<<" "<<srv.request.right_hand_position.z<<" ";
		
		//Write data to file 
		evaluation_data<<motion_plan_num<<" "<<srv.request.right_hand_position.x<<" "<<srv.request.right_hand_position.y<<" "<<srv.request.right_hand_position.z<<" ";
		  
		if(srv.response.success == true)
		{
		  //Write data to file
		  evaluation_data<<1<<" ";
		  evaluation_data<<srv.response.num_nodes_generated<<" ";
		  evaluation_data<<srv.response.search_time<< " ";
		  evaluation_data<<srv.response.time_goal_config<< std::endl;
		  
		  //Publish motion plan trajectory
		  //pub_traj.publish(srv.response.motion_plan);
		  std::cout<< "State: success"<<" ";
		  std::cout<<"Total number of nodes: "<<srv.response.num_nodes_generated<<" ";
		  std::cout<<"Time for planning [sec]: "<<srv.response.search_time<<" ";
		  std::cout<<"Time to gen first goal config [sec]: "<<srv.response.time_goal_config<<std::endl;
		 
		  sleep(1);
		  
		  motion_plan_num++; 
		  
		}
		else
		{
		  //Write data to file
		  evaluation_data<<0<<" "; //failed
		  evaluation_data<<0<<" "; //num_nodes
		  evaluation_data<<0<<" "; //search_time
		  evaluation_data<<srv.response.time_goal_config<< std::endl; //time for first goal config
		  
		  std::cout<< "State: failed"<<" ";
		  std::cout<<"Total number of nodes: "<<0<<" ";
		  std::cout<<"Time for planning [sec]: "<<0<<" ";
		  std::cout<<"Time to gen first goal config [sec]: "<<srv.response.time_goal_config<<std::endl;
		 
		  //std::cout<< "Motion Planning failed !!!"<< std::endl;
		  motion_plan_num++;
		}
	    }
	    else
	    {
	      std::cout<<"Failed to call MOTION PLAN service!"<<std::endl;
	      return 1;
	    }
	  }//END y direction loop
	  
   
      }//END x direction loop
      
      //Close file
      evaluation_data.close();
     
     
   }

   //ros::waitForShutdown();
   return 0;
}






