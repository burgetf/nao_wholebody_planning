 
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

#include <rrt_planner_msgs/Generate_DS_Configs.h>
#include <rrt_planner_msgs/Compute_Goal_Config.h>
#include <rrt_planner_msgs/Compute_Motion_Plan.h>
#include <rrt_planner_msgs/Compute_Linear_Manipulation_Plan.h>
#include <rrt_planner_msgs/Compute_Circular_Manipulation_Plan.h>

//Include Actionlib and messages
#include <nao_msgs/JointTrajectoryAction.h>
#include <nao_msgs/JointAnglesWithSpeedAction.h>
#include <actionlib/client/simple_action_client.h>


#include <std_srvs/Empty.h>




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


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MAIN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
int main(int argc, char **argv)
{
  ros::init(argc, argv, "planning_example_execution_parallel");

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
  
  //Start Monitors of Planning_Scene_Monitor
  psm.startWorldGeometryMonitor();
  psm.startSceneMonitor();
  psm.startStateMonitor();

  //Planning Scene Publisher
  ros::Publisher scene_pub = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::Duration(0.5).sleep();

  //Trajectory publisher
  ros::Publisher pub_traj= nh.advertise<moveit_msgs::DisplayTrajectory>("display_motion_plan", 1);
   
   
  //Create Action Client for executing the trajectory with angleInterpolation
  actionlib::SimpleActionClient<nao_msgs::JointTrajectoryAction> traj_client("joint_trajectory", true);
  // wait for the action server to start
  traj_client.waitForServer(); //will wait for infinite time
  
  //Create Action Client to close hand
  actionlib::SimpleActionClient<nao_msgs::JointAnglesWithSpeedAction> hand_client("joint_angles_action", true);
  // wait for the action server to start
  hand_client.waitForServer(); //will wait for infinite time
  
  //Prepare message for action goal -> CLOSE RIGHT HAND
  nao_msgs::JointAnglesWithSpeed close_hand;
  close_hand.joint_names.push_back("RHand");
  close_hand.joint_angles.push_back(0.0);
  close_hand.speed = 0.3;
  close_hand.relative = 0;
  //Prepare action goal
  nao_msgs::JointAnglesWithSpeedGoal hand_goal;
  hand_goal.joint_angles = close_hand;
  
  //Prepare message for action goal -> RIGHT HAND ALMOST OPEN
  nao_msgs::JointAnglesWithSpeed door_open_hand;
  door_open_hand.joint_names.push_back("RHand");
  door_open_hand.joint_angles.push_back(0.65);
  door_open_hand.speed = 0.3;
  door_open_hand.relative = 0;
  //Prepare action goal
  nao_msgs::JointAnglesWithSpeedGoal hand_door_init;
  hand_door_init.joint_angles = door_open_hand;
  
   //Prepare message for action goal -> RIGHT HAND OPEN
  nao_msgs::JointAnglesWithSpeed drawer_open_hand;
  drawer_open_hand.joint_names.push_back("RHand");
  drawer_open_hand.joint_angles.push_back(1.0);
  drawer_open_hand.speed = 0.3;
  drawer_open_hand.relative = 0;
  //Prepare action goal
  nao_msgs::JointAnglesWithSpeedGoal hand_drawer_init;
  hand_drawer_init.joint_angles = drawer_open_hand;
  
  
  //Service Client nao_controller
  ros::ServiceClient body_stiffness = nh.serviceClient<std_srvs::Empty>("body_stiffness/enable");
  //Enable stiffness
  std_srvs::Empty empty_msgs;
  body_stiffness.call(empty_msgs);
  
  
  
  
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
  
  
  
    //----- Set initial config and publish it onto planning scene
    //planning_models::KinematicState &state = psm.getPlanningScene()->getCurrentState();
    robot_state::RobotState state(psm.getRobotModel());
    state.setToDefaultValues();


    //Get the Name of the Links and Joints of a specific group
    //std::vector<std::string> link_names = psm->getPlanningScene()->getKinematicModel()->getJointModelGroup(PLANNING_GROUP)->getUpdatedLinkModelNames();
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
   //Data for action client
   nao_msgs::JointTrajectoryGoal goal; 
   nao_msgs::JointTrajectoryGoal goal_for_bottle;
   
   if (ACTIVE_SERVICE != 0 && ACTIVE_SERVICE != 1)
   {  
      
      //Drawer Experiment
      if (ACTIVE_SERVICE == 3)
      {
        init_pose[0] = 0.00000;  //RAnkleRoll
        init_pose[1] = 0.0;  //RAnklePitch
        init_pose[2] = 0.0;  //RKneePitch
        init_pose[3] = 0.0;  //RHipPitch
        init_pose[4] = 0.00000;  //RHipRoll
        init_pose[5] = 0.0;  //LShoulderPitch
        init_pose[6] = 0.0;  //LShoulderRoll
        init_pose[7] = 0.0;  //LElbowYaw
        init_pose[8] = -0.5;  //LElbowRoll
        init_pose[9] = 0.00000;  //LWristYaw
        init_pose[10] = 0.0; //RShoulderPitch
        init_pose[11] = 0.0; //RShoulderRoll
        init_pose[12] = 0.0; //RElbowYaw
        init_pose[13] = 0.5; //RElbowRoll
        init_pose[14] = 0.00000; //RWristYaw
        init_pose[15] = 0.00000; //LHipYawPitch
        init_pose[16] = 0.00000; //LHipRoll
        init_pose[17] = 0.0; //LHipPitch
        init_pose[18] = 0.0; //LKneePitch
        init_pose[19] = 0.0; //LAnklePitch
        init_pose[20] = 0.00000; //LAnkleRoll  
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_drawer_init);
        hand_client.waitForResult(ros::Duration(5.0));
      
      }
    
      //Door Experiment
      if (ACTIVE_SERVICE == 4)
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
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_door_init);
        hand_client.waitForResult(ros::Duration(5.0));
      }   
      
      if (ACTIVE_SERVICE == 5)
      {
    
        /*
        init_pose[0] = 0.0757906;  //RAnkleRoll
        init_pose[1] = -0.679186;  //RAnklePitch
        init_pose[2] = 1.31028;  //RKneePitch
        init_pose[3] = -1.03101;  //RHipPitch
        init_pose[4] = 0.146325;  //RHipRoll
        init_pose[5] = 1.5708;  //LShoulderPitch
        init_pose[6] =  0.17;  //LShoulderRoll
        init_pose[7] = 0.0;  //LElbowYaw
        init_pose[8] = -0.036;//-1.04720;  //LElbowRoll
        init_pose[9] = 0.0;  //LWristYaw
        init_pose[10] = 0.356441; //RShoulderPitch
        init_pose[11] = -0.427444; //RShoulderRoll
        init_pose[12] = 1.48403; //RElbowYaw
        init_pose[13] =  1.08452;//1.04720; //RElbowRoll
        init_pose[14] = -0.972506; //RWristYaw
        init_pose[15] = 0.00000; //LHipYawPitch
        init_pose[16] = 0.146922; //LHipRoll
        init_pose[17] = -1.22601; //LHipPitch
        init_pose[18] = 1.62153; //LKneePitch
        init_pose[19] = -0.79544; //LAnklePitch
        init_pose[20] = 0.0752721; //LAnkleRoll
        */
        
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
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_goal);
        hand_client.waitForResult(ros::Duration(5.0));
     
      }
      
   
       //Drawer+Collision Avoidance Experiment
      if (ACTIVE_SERVICE == 6 || ACTIVE_SERVICE == 7)
      {
    
        init_pose[0] = 0.00000;  //RAnkleRoll
        init_pose[1] = -0.349066;  //RAnklePitch
        init_pose[2] = 0.698132;  //RKneePitch
        init_pose[3] = -0.436332;  //RHipPitch
        init_pose[4] = 0.00000;  //RHipRoll
        init_pose[5] = 1.5;  //LShoulderPitch
        init_pose[6] = 0.4;  //LShoulderRoll
        init_pose[7] = -1.39626;  //LElbowYaw
        init_pose[8] = -0.3;//-1.04720;  //LElbowRoll
        init_pose[9] = 0.00000;  //LWristYaw
        init_pose[10] = 1.5; //RShoulderPitch
        init_pose[11] = -0.25; //RShoulderRoll
        init_pose[12] = 1.39626; //RElbowYaw
        init_pose[13] = 0.3;//1.04720; //RElbowRoll
        init_pose[14] = 0.00000; //RWristYaw
        init_pose[15] = 0.00000; //LHipYawPitch
        init_pose[16] = 0.00000; //LHipRoll
        init_pose[17] = -0.436332; //LHipPitch
        init_pose[18] = 0.698132; //LKneePitch
        init_pose[19] = -0.349066; //LAnklePitch
        init_pose[20] = 0.00000; //LAnkleRoll
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_drawer_init);
        hand_client.waitForResult(ros::Duration(5.0));
      }   
      
      
      
      //Set joint values/names of the map
      for (unsigned int i = 0; i != joint_names_.size() ; i++)
      {
        nvalues[joint_names_[i]] = init_pose[i];
        //std::cout<<joint_names_[i] <<std::endl;
      }

      //Set current robot state
      //state.setStateValues(nvalues);
      state.setVariablePositions(nvalues);
      
      
       //Experiment with TABLE collision avoidance
      if (ACTIVE_SERVICE == 2)
      { 
        //--------------------------- Insert Table into Scene -----------------------------------------------
        psm.getPlanningScene()->setName("nao_with_table");
        //create and position Box
        Eigen::Affine3d t;
        t = Eigen::Translation3d(0.35, 0.06, 0.32);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.6, 0.03)), t);
      //--------------------------------------------------------------------------------------------
      }
      
      
      //Experiment with SHELF collision avoidance
      if (ACTIVE_SERVICE == 5)
      { 
        //--------------------------- Insert Shelf into Scene -----------------------------------------------
        //NOTE: Position of shelf is w.r.t right foot frame
        psm.getPlanningScene()->setName("nao_with_shelf");
        //thickness bottom plate
        double thickness_bottom_plate = 0.04;
        double thickness_top_plate = 0.017;
        double thickness_shelfs = 0.014;
        //Shelf edge position
        double shelf_x = 0.33;
        double shelf_y = -0.06;
        double shelf_z = thickness_bottom_plate/2;
        //create bottom plate
        Eigen::Affine3d t;
        t = Eigen::Translation3d(shelf_x, shelf_y, shelf_z);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("plate_bottom", shapes::ShapeConstPtr(new shapes::Box(0.27, 0.31, thickness_bottom_plate)), t);

        //Create left side wall
        t = Eigen::Translation3d(shelf_x + 0.0175, shelf_y + 0.132 , shelf_z + 0.28);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("left_wall", shapes::ShapeConstPtr(new shapes::Box(0.235, 0.014, 0.52)), t);

        //Create right side wall
        t = Eigen::Translation3d(shelf_x + 0.0175, shelf_y - 0.132 , shelf_z + 0.28);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("right_wall", shapes::ShapeConstPtr(new shapes::Box(0.235, 0.014, 0.52)), t);

        //Create top plate
        t = Eigen::Translation3d(shelf_x, shelf_y, shelf_z + 0.5485);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("plate_top", shapes::ShapeConstPtr(new shapes::Box(0.27, 0.31, thickness_top_plate)), t);

        //Create bottom shelf
        t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.147);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("shelf_bottom", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);

        //Create center shelf
        t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.2775);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("shelf_center", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);

        //Create center shelf
        t = Eigen::Translation3d(shelf_x + 0.025, shelf_y, shelf_z + 0.408);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("shelf_center", shapes::ShapeConstPtr(new shapes::Box(0.22, 0.25, thickness_shelfs)), t);
        //--------------------------------------------------------------------------------------------
      }
      
      
      //Experiment with DRAWER + COLLISION AVOIDANCE
      if (ACTIVE_SERVICE == 6)
      { 
        //-//create and position Box (Collision Beam) -----------------------------------------------
        psm.getPlanningScene()->setName("drawer_and_pole");
        //create and position Box
        Eigen::Affine3d t;
        t = Eigen::Translation3d(0.15, -0.08, 0.125);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("beam", shapes::ShapeConstPtr(new shapes::Box(0.06, 0.08, 0.25)), t);
        //--------------------------------------------------------------------------------------------
      
       //create and position Box (Drawer Box)
        Eigen::Affine3d b;
        b = Eigen::Translation3d(0.39, -0.15, 0.25);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("box", shapes::ShapeConstPtr(new shapes::Box(0.16, 0.17, 0.5)), b);
      //--------------------------------------------------------------------------------------------
      
      }
      
      //Experiment Pick and Place Bottle
      if (ACTIVE_SERVICE == 7)
      { 
        //--------------------------- Insert Table into Scene -----------------------------------------------
        psm.getPlanningScene()->setName("nao_with_bottle");
        //create and position Box
        Eigen::Affine3d t;
        t = Eigen::Translation3d(0.34, 0.18, 0.21);
        //psm.getPlanningScene()->getCollisionWorld()->addToObject("pole", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.4, 0.1)), t);
        //--------------------------------------------------------------------------------------------
      
      }
      
      //Publish state on planning scene
      moveit_msgs::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      //psmsg.robot_model_root = "r_sole";
      scene_pub.publish(psmsg);
      sleep(1);
      
      
      
      //Init Trajectory
      trajectory_msgs::JointTrajectory traj_init;
      traj_init.joint_names.resize(joint_names_.size());
      traj_init.points.resize(1);
      traj_init.points[0].positions.resize(joint_names_.size());
      
      for (unsigned int i = 0 ; i < joint_names_.size(); i++)
      {
        traj_init.joint_names[i] = joint_names_[i];
        traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
        //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
        traj_init.points[0].time_from_start = ros::Duration(5.0);
      }
    
      //Send initial trajectory to action server
      goal.trajectory= traj_init;
      goal.relative = 0;
      traj_client.sendGoal(goal);
      traj_client.waitForResult(ros::Duration(5.0)); 
      
         
   }



   //+++++++++++++++++++++++++++++++++++++++++++++++++++ TEST SERVICES ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
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
      srv.request.right_hand_position.x = 0.2;
      srv.request.right_hand_position.y = -0.065;
      srv.request.right_hand_position.z = 0.2;
      srv.request.right_hand_direction.x = 0.0;
      srv.request.right_hand_direction.y = -1.0;
      srv.request.right_hand_direction.z = 0.0;
      
      //Set Start Configuration
      srv.request.wb_start_config = init_pose;

      //Prepare trajectory for execution
      srv.request.execute_trajectory = true;
      
      //Call service 
      if (motion_plan_client.call(srv))
      {
        std::cout<<"Service performed!"<<std::endl;
        if(srv.response.success == true)
        {   
          //----- Execute Trajectory via NAO API wrapper(defined as actions)
          //std::cout<<srv.response.motion_plan.trajectory.joint_trajectory<< std::endl;
          
          //for(unsigned int i = 0; i<srv.response.motion_plan.trajectory.joint_trajectory.points.size(); i++)
          //std::cout<<srv.response.motion_plan.trajectory.joint_trajectory.points[i].time_from_start<<std::endl;
          
          //Get Robot Trajectory from response message
          moveit_msgs::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);
          goal.trajectory= rob_traj.joint_trajectory;
          goal.relative = 0;

          //Send goal trajectory to action server
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(5.0));
          
          //Send goal config (hand closed) to action server
          hand_client.sendGoal(hand_goal);
          hand_client.waitForResult(ros::Duration(5.0));

          std::cout<< "Motion Trajectory published !!!"<< std::endl;

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


      //-------------------------- REPEAT ? ------------------------------
    std::string input;
    std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
    std::getline (std::cin, input);
    char first_element = input[0];

    while (first_element == 'r')
    {

       //Set body stiffness
       body_stiffness.call(empty_msgs);

       //Set initial WB pose
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

       //Send goal config (open hand) to action server
       hand_client.sendGoal(hand_drawer_init);
       hand_client.waitForResult(ros::Duration(5.0));


//       //Set joint values/names of the map
//       for (unsigned int i = 0; i != joint_names_.size() ; i++)
//       {
//         nvalues[joint_names_[i]] = init_pose[i];
//         //std::cout<<joint_names_[i] <<std::endl;
//       }

//       //Set current robot state
//       //state.setStateValues(nvalues);
//       state.setVariablePositions(nvalues);

//       //Publish state on planning scene
//       moveit_msgs::PlanningScene psmsg_init;
//       psm.getPlanningScene()->getPlanningSceneMsg(psmsg_init);
//       //psmsg_init.robot_model_root = "r_sole";
//       scene_pub.publish(psmsg_init);
//       sleep(1);



       //Init Trajectory
       trajectory_msgs::JointTrajectory traj_init;
       traj_init.joint_names.resize(joint_names_.size());
       traj_init.points.resize(1);
       traj_init.points[0].positions.resize(joint_names_.size());

       for (unsigned int i = 0 ; i < joint_names_.size(); i++)
       {
         traj_init.joint_names[i] = joint_names_[i];
         traj_init.points[0].positions[i] = init_pose[i];
         //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
         traj_init.points[0].time_from_start = ros::Duration(5.0);
       }


      //Send initial trajectory to action server
      goal.trajectory= traj_init;
      goal.relative = 0;
      traj_client.sendGoal(goal);
      traj_client.waitForResult(ros::Duration(5.0));


      //------------ Resend Motion Planning Trajectories-----------------------
      //Set action massage data
      moveit_msgs::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);
      goal.trajectory= rob_traj.joint_trajectory;
      goal.relative = 0;

      //Call action server
      traj_client.sendGoal(goal);
      traj_client.waitForResult(ros::Duration(5.0));


      //Send goal config (hand closed) to action server
      hand_client.sendGoal(hand_goal);
      hand_client.waitForResult(ros::Duration(5.0));

      //Ask whether to repeat or to shutdown
      std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
      std::getline (std::cin, input);
      first_element = input[0];

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
      srv_manip.request.object_translation_length = 0.07; //in [m]

      //Set Start Configuration
      srv_manip.request.wb_start_config = init_pose;

      //Prepare trajectory for execution
      srv_manip.request.execute_trajectory = true;
        
      
      //Call service
      if (linear_manipulation_plan_client.call(srv_manip))
      {
        std::cout<<"Service performed!"<<std::endl; 
        
        if(srv_manip.response.success_approach == true)
        {
            //Perform Motion Plan Approach
            //Set action massage data
            moveit_msgs::RobotTrajectory rob_traj = srv_manip.response.motion_plan_approach.trajectory.at(0);
            goal.trajectory= rob_traj.joint_trajectory;
            goal.relative = 0;
            //Call action server
            traj_client.sendGoal(goal);
            traj_client.waitForResult(ros::Duration(8.0));
          
          std::cout<< "Search time approach: "<<srv_manip.response.search_time_approach<< std::endl;
          std::cout<< "Time goal config approach: "<<srv_manip.response.time_goal_config_approach<< std::endl;
          
          std::cout<< "Approach Trajectory published !!!"<< std::endl;
          sleep(1);
        }
        else std::cout<< "Motion Planning Approach failed !!!"<< std::endl;
        
        
        if(srv_manip.response.success_manipulation == true)
        {
          //for(unsigned int i = 0 ; i < srv_manip.response.motion_plan_manipulation.trajectory.joint_trajectory.points.size(); i++)
          //  std::cout<<srv_manip.response.motion_plan_manipulation.trajectory.joint_trajectory.points[i].time_from_start<<std::endl;
          
          
          std::cout<< "Search time manipulate: "<<srv_manip.response.search_time_manipulate<< std::endl;
          std::cout<< "Time goal config manipulate: "<<srv_manip.response.time_goal_config_manipulate<< std::endl;
          
          //Send goal config (hand closed) to action server
          hand_client.sendGoal(hand_goal);
          hand_client.waitForResult(ros::Duration(5.0));
          
          //Set action massage data
          moveit_msgs::RobotTrajectory rob_traj = srv_manip.response.motion_plan_manipulation.trajectory.at(0);
          goal.trajectory= rob_traj.joint_trajectory;
          goal.relative = 0;
          //Call action server
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(8.0));
          
          //Send goal config (open hand) to action server
          hand_client.sendGoal(hand_drawer_init);
          hand_client.waitForResult(ros::Duration(5.0));
          
          //----- Perform a final motion to conclude the manipulation ----- 
          init_pose[0] = 0.00000;  //RAnkleRoll
          init_pose[1] = 0.0;  //RAnklePitch
          init_pose[2] = 0.0;  //RKneePitch
          init_pose[3] = 0.0;  //RHipPitch
          init_pose[4] = 0.00000;  //RHipRoll
          init_pose[5] = 0.3;  //LShoulderPitch
          init_pose[6] = 1.0;  //LShoulderRoll
          init_pose[7] = 0.0;  //LElbowYaw
          init_pose[8] = -0.5;  //LElbowRoll
          init_pose[9] = 0.00000;  //LWristYaw
          init_pose[10] = 0.3; //RShoulderPitch
          init_pose[11] = -1.0; //RShoulderRoll
          init_pose[12] = 0.0; //RElbowYaw
          init_pose[13] = 0.5; //RElbowRoll
          init_pose[14] = 0.00000; //RWristYaw
          init_pose[15] = 0.00000; //LHipYawPitch
          init_pose[16] = 0.00000; //LHipRoll
          init_pose[17] = 0.0; //LHipPitch
          init_pose[18] = 0.0; //LKneePitch
          init_pose[19] = 0.0; //LAnklePitch
          init_pose[20] = 0.00000; //LAnkleRoll 
          
          //Set joint values/names of the map
          for (unsigned int i = 0; i != joint_names_.size() ; i++)
            nvalues[joint_names_[i]] = init_pose[i];
                
          //Final Trajectory
          trajectory_msgs::JointTrajectory traj_init;
          traj_init.joint_names.resize(joint_names_.size());
          traj_init.points.resize(1);
          traj_init.points[0].positions.resize(joint_names_.size());
          
          for (unsigned int i = 0 ; i < joint_names_.size(); i++)
          {
            traj_init.joint_names[i] = joint_names_[i];
            traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
            //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
            traj_init.points[0].time_from_start = ros::Duration(5.0);
          }
            
          //Send final trajectory to action server
          goal.trajectory= traj_init;
          goal.relative = 0;
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(5.0));  
          
          
          //----- Another final motion to conclude the manipulation ----- 
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
          
          
          //Set joint values/names of the map
          for (unsigned int i = 0; i != joint_names_.size() ; i++)
            nvalues[joint_names_[i]] = init_pose[i];
                
          //Final Trajectory
          for (unsigned int i = 0 ; i < joint_names_.size(); i++)
          {
            traj_init.joint_names[i] = joint_names_[i];
            traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
            //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
            traj_init.points[0].time_from_start = ros::Duration(5.0);
          }
            
          //Send final trajectory to action server
          goal.trajectory= traj_init;
          goal.relative = 0;
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(5.0));  
      
          //Manipulation done!!!
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
      
      
       //-------------------------- REPEAT ? ------------------------------
     std::string input;
     std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
     std::getline (std::cin, input);
     char first_element = input[0];
     
     while (first_element == 'r')
     {
       
        //Set body stiffness
        body_stiffness.call(empty_msgs);
       
        init_pose[0] = 0.00000;  //RAnkleRoll
        init_pose[1] = 0.0;  //RAnklePitch
        init_pose[2] = 0.0;  //RKneePitch
        init_pose[3] = 0.0;  //RHipPitch
        init_pose[4] = 0.00000;  //RHipRoll
        init_pose[5] = 0.0;  //LShoulderPitch
        init_pose[6] = 0.0;  //LShoulderRoll
        init_pose[7] = 0.0;  //LElbowYaw
        init_pose[8] = -0.5;  //LElbowRoll
        init_pose[9] = 0.00000;  //LWristYaw
        init_pose[10] = 0.0; //RShoulderPitch
        init_pose[11] = 0.0; //RShoulderRoll
        init_pose[12] = 0.0; //RElbowYaw
        init_pose[13] = 0.5; //RElbowRoll
        init_pose[14] = 0.00000; //RWristYaw
        init_pose[15] = 0.00000; //LHipYawPitch
        init_pose[16] = 0.00000; //LHipRoll
        init_pose[17] = 0.0; //LHipPitch
        init_pose[18] = 0.0; //LKneePitch
        init_pose[19] = 0.0; //LAnklePitch
        init_pose[20] = 0.00000; //LAnkleRoll  
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_drawer_init);
        hand_client.waitForResult(ros::Duration(5.0));
        
        
        //Set joint values/names of the map
        for (unsigned int i = 0; i != joint_names_.size() ; i++)
        {
          nvalues[joint_names_[i]] = init_pose[i];
          //std::cout<<joint_names_[i] <<std::endl;
        }

        //Set current robot state
        //state.setStateValues(nvalues);
        state.setVariablePositions(nvalues);

        //Publish state on planning scene
        moveit_msgs::PlanningScene psmsg_init;
        psm.getPlanningScene()->getPlanningSceneMsg(psmsg_init);
        //psmsg_init.robot_model_root = "r_sole";
        scene_pub.publish(psmsg_init);
        sleep(1);
        
        
        
        //Init Trajectory
        trajectory_msgs::JointTrajectory traj_init;
        traj_init.joint_names.resize(joint_names_.size());
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(joint_names_.size());
        
        for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        {
          traj_init.joint_names[i] = joint_names_[i];
          traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        }
        
        
       //Send initial trajectory to action server
       goal.trajectory= traj_init;
       goal.relative = 0;
       traj_client.sendGoal(goal);
       traj_client.waitForResult(ros::Duration(5.0)); 
        
        
      //------------ Motion Planning Trajectories-----------------------
      
      //Send initial trajectory to action server
      goal.trajectory= traj_init;
      goal.relative = 0;
      traj_client.sendGoal(goal);
      traj_client.waitForResult(ros::Duration(5.0)); 
        
        
       //Set action massage data
        moveit_msgs::RobotTrajectory rob_traj = srv_manip.response.motion_plan_approach.trajectory.at(0);
        goal.trajectory= rob_traj.joint_trajectory;
        goal.relative = 0;

        //Call action server
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));
        
        
        //Send goal config (hand closed) to action server
        hand_client.sendGoal(hand_goal);
        hand_client.waitForResult(ros::Duration(5.0));
        
        //Set action massage data
        rob_traj = srv_manip.response.motion_plan_manipulation.trajectory.at(0);
        goal.trajectory= rob_traj.joint_trajectory;
        goal.relative = 0;

        //Call action server
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_drawer_init);
        hand_client.waitForResult(ros::Duration(5.0));
        
        
         //----- Perform final motion to conclude the manipulation ----- 
        //Map containing joint names and values
        std::map<std::string, double> nvalues_2;
         //----- Perform a final motion to conclude the manipulation ----- 
        init_pose[10] = 0.3; //RShoulderPitch
        init_pose[11] = -1.2; //RShoulderRoll
        init_pose[12] = 0.0; //RElbowYaw
        init_pose[13] = 0.5; //RElbowRoll
        
        //Set joint values/names of the map
        //for (unsigned int i = 0; i != joint_names_.size() ; i++)
        nvalues_2[joint_names_[11]] = init_pose[11];
              
        //Final Trajectory
        traj_init.joint_names.resize(1);
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(1);
        
        //for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        //{
          traj_init.joint_names[0] = joint_names_[11];
          traj_init.points[0].positions[0] = nvalues_2[joint_names_[11]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        //}
          
        //Send final trajectory to action server
        goal.trajectory= traj_init;
        goal.relative = 0;
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));  
        
        
        
        //----- Perform a final motion to conclude the manipulation ----- 
        init_pose[0] = 0.00000;  //RAnkleRoll
        init_pose[1] = 0.0;  //RAnklePitch
        init_pose[2] = 0.0;  //RKneePitch
        init_pose[3] = 0.0;  //RHipPitch
        init_pose[4] = 0.00000;  //RHipRoll
        init_pose[5] = 0.3;  //LShoulderPitch
        init_pose[6] = 1.0;  //LShoulderRoll
        init_pose[7] = 0.0;  //LElbowYaw
        init_pose[8] = -0.5;  //LElbowRoll
        init_pose[9] = 0.00000;  //LWristYaw
        init_pose[10] = 0.3; //RShoulderPitch
        init_pose[11] = -1.0; //RShoulderRoll
        init_pose[12] = 0.0; //RElbowYaw
        init_pose[13] = 0.5; //RElbowRoll
        init_pose[14] = 0.00000; //RWristYaw
        init_pose[15] = 0.00000; //LHipYawPitch
        init_pose[16] = 0.00000; //LHipRoll
        init_pose[17] = 0.0; //LHipPitch
        init_pose[18] = 0.0; //LKneePitch
        init_pose[19] = 0.0; //LAnklePitch
        init_pose[20] = 0.00000; //LAnkleRoll 
        
        traj_init.joint_names.resize(joint_names_.size());
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(joint_names_.size());
        
         //Set joint values/names of the map
        for (unsigned int i = 0; i != joint_names_.size() ; i++)
          nvalues[joint_names_[i]] = init_pose[i];
              
        //Final Trajectory
        for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        {
          traj_init.joint_names[i] = joint_names_[i];
          traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        }
          
        //Send final trajectory to action server
        goal.trajectory= traj_init;
        goal.relative = 0;
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));  
        
        
        
        //----- Another final motion to conclude the manipulation ----- 
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
        
        
        //Set joint values/names of the map
        for (unsigned int i = 0; i != joint_names_.size() ; i++)
          nvalues[joint_names_[i]] = init_pose[i];
              
        //Final Trajectory
        for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        {
          traj_init.joint_names[i] = joint_names_[i];
          traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        }
          
        //Send final trajectory to action server
        goal.trajectory= traj_init;
        goal.relative = 0;
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0)); 
        
        std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
        std::getline (std::cin, input);
        first_element = input[0];
        
     }  
     // END REPEAT
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
      srv_manip.request.right_hand_position.z = 0.35; //0.28
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

      //Prepare trajectory for execution
      srv_manip.request.execute_trajectory = true;
      
      
      //Call service
      if (circular_manipulation_plan_client.call(srv_manip))
      {
      std::cout<<"Service performed!"<<std::endl;
      //Approach
      if(srv_manip.response.success_approach == true)
      {
        std::cout<< "Search time approach: "<<srv_manip.response.search_time_approach<< std::endl;
        std::cout<< "Time goal config approach: "<<srv_manip.response.time_goal_config_approach<< std::endl;
         
        //Set action massage data
        moveit_msgs::RobotTrajectory rob_traj = srv_manip.response.motion_plan_approach.trajectory.at(0);
        goal.trajectory= rob_traj.joint_trajectory;
        goal.relative = 0;
        //Call action server
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));
        
        std::cout<< "Approach Trajectory published !!!"<< std::endl;
        sleep(1);
      }
      else std::cout<< "Motion Planning Approach failed !!!"<< std::endl;
      
      //Manipulation
      if(srv_manip.response.success_manipulation == true)
      {
         std::cout<< "Search time manipulate: "<<srv_manip.response.search_time_manipulate<< std::endl;
         std::cout<< "Time goal config manipulate: "<<srv_manip.response.time_goal_config_manipulate<< std::endl;
         
        //for(unsigned int i = 0 ; i < srv_manip.response.motion_plan_manipulation.trajectory.joint_trajectory.points.size(); i++)
        //  std::cout<<srv_manip.response.motion_plan_manipulation.trajectory.joint_trajectory.points[i].time_from_start<<std::endl;
        
        //Send goal config (hand closed) to action server
        hand_client.sendGoal(hand_goal);
        hand_client.waitForResult(ros::Duration(5.0));
        
        //Set action massage data
        moveit_msgs::RobotTrajectory rob_traj = srv_manip.response.motion_plan_manipulation.trajectory.at(0);
        goal.trajectory= rob_traj.joint_trajectory;
        goal.relative = 0;
        //Call action server
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_drawer_init);
        hand_client.waitForResult(ros::Duration(5.0));
        
        
         //----- Perform final motion to conclude the manipulation ----- 
        //Map containing joint names and values
        std::map<std::string, double> nvalues_2;
         //----- Perform a final motion to conclude the manipulation ----- 
        init_pose[10] = 0.3; //RShoulderPitch
        init_pose[11] = -1.2; //RShoulderRoll
        init_pose[12] = 0.0; //RElbowYaw
        init_pose[13] = 0.5; //RElbowRoll
        
        //Set joint values/names of the map
        //for (unsigned int i = 0; i != joint_names_.size() ; i++)
        nvalues_2[joint_names_[11]] = init_pose[11];
              
        //Final Trajectory
        trajectory_msgs::JointTrajectory traj_init;
        traj_init.joint_names.resize(1);
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(1);
        
        //for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        //{
          traj_init.joint_names[0] = joint_names_[11];
          traj_init.points[0].positions[0] = nvalues_2[joint_names_[11]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        //}
          
        //Send final trajectory to action server
        goal.trajectory= traj_init;
        goal.relative = 0;
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));  
        
        
        
          //----- Another final motion to conclude the manipulation ----- 
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
        
        traj_init.joint_names.resize(joint_names_.size());
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(joint_names_.size());
        
         //Set joint values/names of the map
        for (unsigned int i = 0; i != joint_names_.size() ; i++)
          nvalues[joint_names_[i]] = init_pose[i];
              
        //Final Trajectory
        for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        {
          traj_init.joint_names[i] = joint_names_[i];
          traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        }
          
        //Send final trajectory to action server
        goal.trajectory= traj_init;
        goal.relative = 0;
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));  
        
        
        
        
        //Manipulation done!!!
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
      
      
     //-------------------------- REPEAT ? ------------------------------
     std::string input;
     std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
     std::getline (std::cin, input);
     char first_element = input[0];
     
     while (first_element == 'r')
     {
       
       //Set body stiffness
       body_stiffness.call(empty_msgs);
       
       //Init Trajectory
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
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_door_init);
        hand_client.waitForResult(ros::Duration(5.0));
        
        
        //Set joint values/names of the map
        for (unsigned int i = 0; i != joint_names_.size() ; i++)
        {
          nvalues[joint_names_[i]] = init_pose[i];
          //std::cout<<joint_names_[i] <<std::endl;
        }

        //Set current robot state
        //state.setStateValues(nvalues);
        state.setVariablePositions(nvalues);
        
        //Publish state on planning scene
        moveit_msgs::PlanningScene psmsg_init;
        psm.getPlanningScene()->getPlanningSceneMsg(psmsg_init);
        //psmsg_init.robot_model_root = "r_sole";
        scene_pub.publish(psmsg_init);
        sleep(1);
        
        
        
        //Init Trajectory
        trajectory_msgs::JointTrajectory traj_init;
        traj_init.joint_names.resize(joint_names_.size());
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(joint_names_.size());
        
        for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        {
          traj_init.joint_names[i] = joint_names_[i];
          traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        }
        
        
       //Send initial trajectory to action server
       goal.trajectory= traj_init;
       goal.relative = 0;
       traj_client.sendGoal(goal);
       traj_client.waitForResult(ros::Duration(5.0)); 
        
        
      //------------ Motion Planning Trajectories-----------------------
      
      //Send initial trajectory to action server
      goal.trajectory= traj_init;
      goal.relative = 0;
      traj_client.sendGoal(goal);
      traj_client.waitForResult(ros::Duration(5.0)); 
        
        
       //Set action massage data
       moveit_msgs::RobotTrajectory rob_traj = srv_manip.response.motion_plan_approach.trajectory.at(0);
       goal.trajectory= rob_traj.joint_trajectory;
       goal.relative = 0;

        //Call action server
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));
        
        
        //Send goal config (hand closed) to action server
        hand_client.sendGoal(hand_goal);
        hand_client.waitForResult(ros::Duration(5.0));
        
        //Set action massage data
        rob_traj = srv_manip.response.motion_plan_manipulation.trajectory.at(0);
        goal.trajectory= rob_traj.joint_trajectory;
        goal.relative = 0;


        //Call action server
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));
        
        //Send goal config (open hand) to action server
        hand_client.sendGoal(hand_drawer_init);
        hand_client.waitForResult(ros::Duration(5.0));
        
        
         //----- Perform final motion to conclude the manipulation ----- 
        //Map containing joint names and values
        std::map<std::string, double> nvalues_2;
         //----- Perform a final motion to conclude the manipulation ----- 
        init_pose[10] = 0.3; //RShoulderPitch
        init_pose[11] = -1.2; //RShoulderRoll
        init_pose[12] = 0.0; //RElbowYaw
        init_pose[13] = 0.5; //RElbowRoll
        
        //Set joint values/names of the map
        //for (unsigned int i = 0; i != joint_names_.size() ; i++)
        nvalues_2[joint_names_[11]] = init_pose[11];
              
        //Final Trajectory
        traj_init.joint_names.resize(1);
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(1);
        
        //for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        //{
          traj_init.joint_names[0] = joint_names_[11];
          traj_init.points[0].positions[0] = nvalues_2[joint_names_[11]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        //}
          
        //Send final trajectory to action server
        goal.trajectory= traj_init;
        goal.relative = 0;
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));  
        
        
        
          //----- Another final motion to conclude the manipulation ----- 
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
        
        traj_init.joint_names.resize(joint_names_.size());
        traj_init.points.resize(1);
        traj_init.points[0].positions.resize(joint_names_.size());
        
         //Set joint values/names of the map
        for (unsigned int i = 0; i != joint_names_.size() ; i++)
          nvalues[joint_names_[i]] = init_pose[i];
              
        //Final Trajectory
        for (unsigned int i = 0 ; i < joint_names_.size(); i++)
        {
          traj_init.joint_names[i] = joint_names_[i];
          traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
          //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
          traj_init.points[0].time_from_start = ros::Duration(5.0);
        }
          
        //Send final trajectory to action server
        goal.trajectory= traj_init;
        goal.relative = 0;
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));  
        
        std::cout << "Press <r> to replay the trajectory or <s> to shutdown: ";
        std::getline (std::cin, input);
        first_element = input[0];
        
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
      
      //Set Start Configuration
      srv.request.wb_start_config = init_pose;

      //Prepare trajectory for execution
      srv.request.execute_trajectory = true;
      
      
       //Send goal config (hand closed) to action server
      hand_client.sendGoal(hand_goal);
      hand_client.waitForResult(ros::Duration(5.0));
        
        //Call service 
      if (motion_plan_client.call(srv))
      {
      std::cout<<"Service performed!"<<std::endl;
      if(srv.response.success == true)
      { 
        
        std::cout<<"Upper shelf reached"<<std::endl;
        
        std::cout<<"Time to gen first goal config [sec]: "<<srv.response.time_goal_config<<std::endl;
        
         //PERFORM FIRST MOTION PLAN
        moveit_msgs::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);
        goal.trajectory= rob_traj.joint_trajectory;
        goal.relative = 0;
        
        //Time required for search
        std::cout<<"Time for planning: "<<srv.response.search_time<<std::endl;
        
        //Send goal trajectory to action server
        traj_client.sendGoal(goal);
        traj_client.waitForResult(ros::Duration(5.0));    
  
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

      //Get current state
      //planning_models::KinematicState &new_state = psm.getPlanningScene()->getCurrentState();
      robot_state::RobotState new_state(psm.getRobotModel());
      new_state.setToDefaultValues();
      //robot_state::RobotState &new_state = psm.getPlanningScene()->getCurrentState();


      //Get Robot Trajectory from response message
      moveit_msgs::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);
      //Get corresponding joint_trajectory
      trajectory_msgs::JointTrajectory joint_traj = rob_traj.joint_trajectory;
      //Get last element of trajectory
      int last_element_index = joint_traj.points.size()-1;


      //Set joint values/names of the map
      //int last_element_index = srv.response.motion_plan.trajectory.joint_trajectory.points.size()-1;
      for (unsigned int i = 0; i < joint_names_.size() ; i++)
      {
        new_values[joint_names_[i]] = joint_traj.points[last_element_index].positions[i];
        //new_values[joint_names_[i]] = srv.response.motion_plan.trajectory.joint_trajectory.points[last_element_index].positions[i];
        //std::cout<<joint_names_[i]<<": "<<srv.response.motion_plan.trajectory.joint_trajectory.points[last_element_index].positions[i]<<std::endl;
      }
      //Set current robot state
      //new_state.setStateValues(new_values);
      new_state.setVariablePositions(new_values);

      //Apply robot state to planning scene
      psm.getPlanningScene()->setCurrentState(new_state);

      //std::cout<<"Published on planning scene" <<std::endl;
      // for (unsigned int i = 0; i < joint_names_.size() ; i++)
      //{
      //    std::cout<<joint_names_[i]<<": "<<new_values[joint_names_[i]]<<std::endl;
      //}
      
      //Publish state on planning scene
      moveit_msgs::PlanningScene psmsg_2;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg_2);
      //psmsg_2.robot_model_root = "r_sole";
      scene_pub.publish(psmsg_2);
      sleep(4);
      
      
      //----------------------------------------------------
      
      
      
      //Search for second motion plan
      //Service request and response for second motion plan
      rrt_planner_msgs::Compute_Motion_Plan srv_2;
      //-----Put Hand into lower shelf
      srv_2.request.right_hand_position.x = 0.25;
      srv_2.request.right_hand_position.y = -0.08;
      srv_2.request.right_hand_position.z = 0.22;
      srv_2.request.right_hand_direction.x = 0.0;
      srv_2.request.right_hand_direction.y = -1.0;
      srv_2.request.right_hand_direction.z = 0.0;  
      
      
      //Prepare trajectory for execution
      srv_2.request.execute_trajectory = true;
      
      
      //Call service 
      if (motion_plan_client.call(srv_2))
      {
        std::cout<<"Service performed!"<<std::endl;
        
        if(srv_2.response.success == true)
        { 
          
          std::cout<<"Lower shelf reached"<<std::endl;
        
          std::cout<<"Time to gen first goal config [sec]: "<<srv_2.response.time_goal_config<<std::endl;
          //for (unsigned int i = 0; i < joint_names_.size() ; i++)
          //{
          //  std::cout<<joint_names_[i]<<": "<<srv.response.motion_plan.trajectory.joint_trajectory.points[0].positions[i]<<std::endl;
          //}
        
          //Time required for search
          std::cout<<"Time for planning: "<<srv_2.response.search_time<<std::endl;
          
          //PERFORM SECOND MOTION PLAN
          moveit_msgs::RobotTrajectory rob_traj_2 = srv_2.response.motion_plan.trajectory.at(0);
          goal.trajectory= rob_traj_2.joint_trajectory;
          goal.relative = 0;
          
          //Send goal trajectory to action server
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(5.0));
    
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


  
  //--------------------------- Test: DRAWER WITH COLLISION AVOIDANCE -----------------------
   if (ACTIVE_SERVICE == 6)
   {
     
      //Service Client
      ros::ServiceClient linear_manipulation_plan_client = nh.serviceClient<rrt_planner_msgs::Compute_Linear_Manipulation_Plan>("compute_linear_manipulation_plan");
      //Service request and response
      rrt_planner_msgs::Compute_Linear_Manipulation_Plan srv_manip;
      //Fill request with data
      srv_manip.request.right_hand_position.x = 0.25;
      srv_manip.request.right_hand_position.y = -0.065;
      srv_manip.request.right_hand_position.z = 0.30;
      srv_manip.request.right_hand_direction.x = 0.0;
      srv_manip.request.right_hand_direction.y = -1.0;
      srv_manip.request.right_hand_direction.z = 0.0;
      //Object data
      srv_manip.request.relative_angle = 0.0; //in degrees
      srv_manip.request.object_translation_length = 0.07; //in [m]

      //Set Start Configuration
      srv_manip.request.wb_start_config = init_pose;

      //Prepare trajectory for execution
      srv_manip.request.execute_trajectory = true;
        
      
      //Call service
      if (linear_manipulation_plan_client.call(srv_manip))
      {
        std::cout<<"Service performed!"<<std::endl; 
        
        if(srv_manip.response.success_approach == true)
        {
            //Perform Motion Plan Approach
            //Set action massage data
            moveit_msgs::RobotTrajectory rob_traj = srv_manip.response.motion_plan_approach.trajectory.at(0);
            goal.trajectory= rob_traj.joint_trajectory;
            goal.relative = 0;
            //Call action server
            traj_client.sendGoal(goal);
            traj_client.waitForResult(ros::Duration(8.0));
          
          std::cout<< "Search time approach: "<<srv_manip.response.search_time_approach<< std::endl;
          std::cout<< "Time goal config approach: "<<srv_manip.response.time_goal_config_approach<< std::endl;
          std::cout<< "Num nodes approach: "<<srv_manip.response.num_nodes_generated_approach<< std::endl;
          
          std::cout<< "Approach Trajectory published !!!"<< std::endl;
          sleep(1);
        }
        else std::cout<< "Motion Planning Approach failed !!!"<< std::endl;
        
        
        if(srv_manip.response.success_manipulation == true)
        {
          //for(unsigned int i = 0 ; i < srv_manip.response.motion_plan_manipulation.trajectory.joint_trajectory.points.size(); i++)
          //  std::cout<<srv_manip.response.motion_plan_manipulation.trajectory.joint_trajectory.points[i].time_from_start<<std::endl;
          
          
          std::cout<< "Search time manipulate: "<<srv_manip.response.search_time_manipulate<< std::endl;
          std::cout<< "Time goal config manipulate: "<<srv_manip.response.time_goal_config_manipulate<< std::endl;
          std::cout<< "Num nodes approach: "<<srv_manip.response.num_nodes_generated_manipulate<< std::endl;
          
          //Send goal config (hand closed) to action server
          hand_client.sendGoal(hand_goal);
          hand_client.waitForResult(ros::Duration(5.0));
          
          //Set action massage data
          moveit_msgs::RobotTrajectory rob_traj_2 = srv_manip.response.motion_plan_manipulation.trajectory.at(0);
          goal.trajectory= rob_traj_2.joint_trajectory;
          goal.relative = 0;

          //Call action server
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(8.0));
          
          //Send goal config (open hand) to action server
          hand_client.sendGoal(hand_drawer_init);
          hand_client.waitForResult(ros::Duration(5.0));
          
          
          //----- Perform a final motion to conclude the manipulation ----- 
          init_pose[0] = 0.00000;  //RAnkleRoll
          init_pose[1] = 0.0;  //RAnklePitch
          init_pose[2] = 0.0;  //RKneePitch
          init_pose[3] = 0.0;  //RHipPitch
          init_pose[4] = 0.00000;  //RHipRoll
          init_pose[5] = 0.3;  //LShoulderPitch
          init_pose[6] = 1.5;  //LShoulderRoll
          init_pose[7] = 0.0;  //LElbowYaw
          init_pose[8] = -0.5;  //LElbowRoll
          init_pose[9] = 0.00000;  //LWristYaw
          init_pose[10] = 0.3; //RShoulderPitch
          init_pose[11] = -1.5; //RShoulderRoll
          init_pose[12] = 0.0; //RElbowYaw
          init_pose[13] = 0.5; //RElbowRoll
          init_pose[14] = 0.00000; //RWristYaw
          init_pose[15] = 0.00000; //LHipYawPitch
          init_pose[16] = 0.00000; //LHipRoll
          init_pose[17] = 0.0; //LHipPitch
          init_pose[18] = 0.0; //LKneePitch
          init_pose[19] = 0.0; //LAnklePitch
          init_pose[20] = 0.00000; //LAnkleRoll 
          
          //Set joint values/names of the map
          for (unsigned int i = 0; i != joint_names_.size() ; i++)
            nvalues[joint_names_[i]] = init_pose[i];
                
          //Final Trajectory
          trajectory_msgs::JointTrajectory traj_init;
          traj_init.joint_names.resize(joint_names_.size());
          traj_init.points.resize(1);
          traj_init.points[0].positions.resize(joint_names_.size());
          
          for (unsigned int i = 0 ; i < joint_names_.size(); i++)
          {
            traj_init.joint_names[i] = joint_names[i];
            traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
            //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
            traj_init.points[0].time_from_start = ros::Duration(5.0);
          }
            
          //Send final trajectory to action server
          goal.trajectory= traj_init;
          goal.relative = 0;
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(5.0));  
          
          
          //----- Another final motion to conclude the manipulation ----- 
          init_pose[0] = 0.00000;  //RAnkleRoll
          init_pose[1] = -0.349066;  //RAnklePitch
          init_pose[2] = 0.698132;  //RKneePitch
          init_pose[3] = -0.436332;  //RHipPitch
          init_pose[4] = 0.00000;  //RHipRoll
          init_pose[5] = 1.5;  //LShoulderPitch
          init_pose[6] = 0.20;  //LShoulderRoll
          init_pose[7] = -1.39626;  //LElbowYaw
          init_pose[8] = -0.3;//-1.04720;  //LElbowRoll
          init_pose[9] = 0.00000;  //LWristYaw
          init_pose[10] = 1.5; //RShoulderPitch
          init_pose[11] = -0.20; //RShoulderRoll
          init_pose[12] = 1.39626; //RElbowYaw
          init_pose[13] = 0.3;//1.04720; //RElbowRoll
          init_pose[14] = 0.00000; //RWristYaw
          init_pose[15] = 0.00000; //LHipYawPitch
          init_pose[16] = 0.00000; //LHipRoll
          init_pose[17] = -0.436332; //LHipPitch
          init_pose[18] = 0.698132; //LKneePitch
          init_pose[19] = -0.349066; //LAnklePitch
          init_pose[20] = 0.00000; //LAnkleRoll
          
          
          //Set joint values/names of the map
          for (unsigned int i = 0; i != joint_names_.size() ; i++)
            nvalues[joint_names_[i]] = init_pose[i];
                
          //Final Trajectory
          for (unsigned int i = 0 ; i < joint_names_.size(); i++)
          {
            traj_init.joint_names[i] = joint_names_[i];
            traj_init.points[0].positions[i] = nvalues[joint_names_[i]];
            //std::cout<<traj_init.joint_names[i] <<": "<<traj_init.points[0].positions[i]<< std::endl;
            traj_init.points[0].time_from_start = ros::Duration(5.0);
          }
            
          //Send final trajectory to action server
          goal.trajectory= traj_init;
          goal.relative = 0;
          traj_client.sendGoal(goal);
          traj_client.waitForResult(ros::Duration(5.0));  
      
          //Manipulation done!!!
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
   
   
   
   //--------------------------- Test: PICK AND PLACE FOR A BOTTLE -----------------------
   if (ACTIVE_SERVICE == 7)
   {
      //--------------------- FIRST MOTION PLAN -------------------------------
      //Service Client
      ros::ServiceClient motion_plan_client = nh.serviceClient<rrt_planner_msgs::Compute_Motion_Plan>("compute_motion_plan");
      //Service request and response
      rrt_planner_msgs::Compute_Motion_Plan srv;
      //Fill request with data
      srv.request.right_hand_position.x = 0.23;
      srv.request.right_hand_position.y = 0.03;
      srv.request.right_hand_position.z = 0.33;
      srv.request.right_hand_direction.x = 0.0;
      srv.request.right_hand_direction.y = 0.0;
      srv.request.right_hand_direction.z = -1.0;
      
      //Set Start Configuration
      srv.request.wb_start_config = init_pose;

      //Prepare trajectory for execution
      srv.request.execute_trajectory = true;
      
      //Call service 
      if (motion_plan_client.call(srv))
      {
        std::cout<<"Service performed!"<<std::endl;
        
        if(srv.response.success == true)
        {           
           //Execute first motion plan
           moveit_msgs::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);
           goal.trajectory= rob_traj.joint_trajectory;
           goal.relative = 0;
          
           //Send goal trajectory to action server
           traj_client.sendGoal(goal);
           traj_client.waitForResult(ros::Duration(5.0));
          
           //Send goal config (hand closed) to action server
           hand_client.sendGoal(hand_goal);
           hand_client.waitForResult(ros::Duration(5.0));
           
           std::cout<< "Motion Trajectory GRASP published !!!"<< std::endl;

           std::cout<< "Search time approach: "<<srv.response.search_time<< std::endl;
           std::cout<< "Time goal config approach: "<<srv.response.time_goal_config<< std::endl;
           std::cout<< "Num nodes approach: "<<srv.response.num_nodes_generated<< std::endl;
           
           //------------- Perform Lift off Trajectory -----------
           //Map containing joint names and values
           //std::map<std::string, double> nvalues_2;
           //----- Lift off motion ----- 
           //init_pose[10] =  0.2; //RShoulderPitch
           //init_pose[11] = -0.15; //RShoulderRoll
           
           //nvalues_2[joint_names_[10]] = init_pose[10];
           //nvalues_2[joint_names_[11]] = init_pose[11];
            
            
           //Lift off Trajectory
           //trajectory_msgs::JointTrajectory traj_lift_off;
           //traj_lift_off.joint_names.resize(2);
           //traj_lift_off.points.resize(1);
           //traj_lift_off.points[0].positions.resize(2);
        
           //traj_lift_off.joint_names[0] = joint_names_[10];
           //traj_lift_off.joint_names[1] = joint_names_[11];
           //traj_lift_off.points[0].positions[0] = nvalues_2[joint_names_[10]];
           //traj_lift_off.points[0].positions[1] = nvalues_2[joint_names_[11]];
           //traj_lift_off.points[0].time_from_start = ros::Duration(5.0);
        
          
           //Send final trajectory to action server
           //goal.trajectory= traj_lift_off;
           //goal.relative = 0;
           //traj_client.sendGoal(goal);
           //traj_client.waitForResult(ros::Duration(5.0));


           //--------------------- SECOND MOTION PLAN -------------------------------
           //Search for second motion plan
           //Service request and response for second motion plan
           rrt_planner_msgs::Compute_Motion_Plan srv_2;
           //-----Put Hand into lower shelf
           srv_2.request.right_hand_position.x = 0.22;
           srv_2.request.right_hand_position.y = -0.1;
           srv_2.request.right_hand_position.z = 0.26;
           srv_2.request.right_hand_direction.x = 0.0;
           srv_2.request.right_hand_direction.y = -1.0;
           srv_2.request.right_hand_direction.z = 0.0;

           //Set Start Configuration to final configuration of previous motion plan
           srv_2.request.wb_start_config = srv.response.wb_goal_config;

           //Prepare trajectory for execution
           srv_2.request.execute_trajectory = true;


           //Call service
           if (motion_plan_client.call(srv_2))
           {
             std::cout<<"Service performed!"<<std::endl;

             if (srv_2.response.success == true)
             {
                 //Execute second motion plan
                 moveit_msgs::RobotTrajectory rob_traj = srv_2.response.motion_plan.trajectory.at(0);
                 goal_for_bottle.trajectory= rob_traj.joint_trajectory;
                 goal_for_bottle.relative = 0;

                 //Send goal trajectory to action server
                 traj_client.sendGoal(goal_for_bottle);
                 traj_client.waitForResult(ros::Duration(5.0));


                 //Send goal config (open hand) to action server
                 hand_client.sendGoal(hand_drawer_init);
                 hand_client.waitForResult(ros::Duration(5.0));

                 std::cout<< "Motion Trajectory TRANSFER published !!!"<< std::endl;

                 std::cout<< "Planning time manip: "<<srv_2.response.search_time<< std::endl;
                 std::cout<< "Time goal config manip: "<<srv_2.response.time_goal_config<< std::endl;
                 std::cout<< "Num nodes manip: "<<srv_2.response.num_nodes_generated<< std::endl;
             }
             else
             {
                 std::cout<< "Motion Planning TRANSFER failed !!!"<< std::endl;
             }

           }
           else
           {
             std::cout<<"Failed to call MOTION PLAN service!"<<std::endl;
             return 1;
           }

    
        }
        else
        {
          std::cout<< "Motion Planning GRASP failed !!!"<< std::endl;
        }

      }
      else
      {
        std::cout<<"Failed to call MOTION PLAN service!"<<std::endl;
        return 1;
      }
      
      
      
//       //----------------------- Update Planning Scene for next motion plan -------------------
//      //Map containing joint names and values
//      std::map<std::string, double> new_values;
      
//      //Get current state
//      //planning_models::KinematicState &new_state = psm.getPlanningScene()->getCurrentState();
//      robot_state::RobotState new_state(psm.getRobotModel());
      
//      //Set joint values/names of the map
//      moveit_msgs::RobotTrajectory rob_traj = srv.response.motion_plan.trajectory.at(0);
//      int last_element_index = rob_traj.joint_trajectory.points.size()-1;




//      for (unsigned int i = 0; i < joint_names_.size() ; i++)
//      {
//        new_values[joint_names_[i]] = rob_traj.joint_trajectory.points[last_element_index].positions[i];
//        std::cout<<joint_names_[i]<<": "<<rob_traj.joint_trajectory.points[last_element_index].positions[i]<<std::endl;
//      }
//      std::cout<<std::endl;
       
//     //Set current robot state
//     //new_state.setStateValues(new_values);
//     new_state.setVariablePositions(new_values);
      
      
//      Eigen::Affine3d k;
//      k = Eigen::Translation3d(0.34, -0.03, 0.24);
//      //psm.getPlanningScene()->getCollisionWorld()->addToObject("addBox", shapes::ShapeConstPtr(new shapes::Box(0.3, 0.02, 0.1)), k);
      
      
//      //Publish state on planning scene
//      moveit_msgs::PlanningScene psmsg_2;
//      psm.getPlanningScene()->getPlanningSceneMsg(psmsg_2);
//      //psmsg_2.robot_model_root = "r_sole";
//      scene_pub.publish(psmsg_2);
//      sleep(4);
      
   }
   
   
   //ros::waitForShutdown();
   return 0;
}





