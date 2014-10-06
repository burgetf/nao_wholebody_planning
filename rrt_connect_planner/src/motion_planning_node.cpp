/*
 * motion_planning_node.cpp
 *
 *  Created on: Aug 14, 2012
 *      Author: Felix Burget
 */


#include <rrt_connect_planner/planning_service_center.h>
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planning_node");

  planner_control::Planning_Service_Center planning_service;

  ros::spin();

  return 0;
}
