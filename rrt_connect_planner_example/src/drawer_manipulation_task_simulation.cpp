
/*
 * planning_example.cpp
 *
 *  Created on: April 25, 2014
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
  ros::init(argc, argv, "drawer_manipulation_task");

  ros::NodeHandle nh;

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


    //Define init poses for drawer manipulation
    std::vector<double> init_pose(21);
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

    //Map containing joint names and values
    std::map<std::string, double> nvalues;



      //Set joint values/names of the map
      for (unsigned int i = 0; i != joint_names_.size() ; i++)
      {
        nvalues[joint_names_[i]] = init_pose[i];
        std::cout<<joint_names_[i] <<std::endl;
      }

      //Set current robot state
      state.setVariablePositions(nvalues);

      //Apply robot state to planning scene
      psm.getPlanningScene()->setCurrentState(state);


      //Publish state on planning scene
      moveit_msgs::PlanningScene psmsg;
      psm.getPlanningScene()->getPlanningSceneMsg(psmsg);
      scene_pub.publish(psmsg);
      sleep(1);





   //--------------------------- Test: LINEAR MANIPULATION PLAN Service -----------------------
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



   return 0;
}






