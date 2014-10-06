 /*
 * planning_service_center.h
 *
 *  Created on: Aug 14, 2012
 *      Author: Felix Burget
 */

#ifndef PLANNING_SERVICE_CENTER_H_
#define PLANNING_SERVICE_CENTER_H_

#include <rrt_connect_planner/nao_planner_control.h>

namespace planner_control {

class Planning_Service_Center
{
	public:
	Planning_Service_Center();
	~Planning_Service_Center();



	protected:

	//nao_planner_control object
	NAO_PLANNER_CONTROL nao_planner_control;

	//Services
	ros::ServiceServer generate_ds_database_;
	ros::ServiceServer get_goal_config_;
	ros::ServiceServer motion_planning_;
	ros::ServiceServer linear_manipulation_planning_;
	ros::ServiceServer circular_manipulation_planning_;

};




}
#endif /* PLANNING_SERVICE_CENTER_H_ */
