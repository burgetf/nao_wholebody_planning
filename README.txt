Steps to run Whole-Body Motion Planner:

//----------------------------------------------- SETUP -------------------------------------------------------
1) roslaunch rrt_connect_planner planning_context_rfoot.launch (starts all planner services and sets parameters on param server)
2) rosrun rviz rviz (optional)

//-------------------------------------------- EXAMPLES -------------------------------------------------------
3.1) For a simulated NAO:
  roslaunch rrt_connect_planner_example nao_local.launch (for a simulated NAO / NAOqi needs to be started beforehand)
  rosrun rrt_connect_planner_example planning_example_simulation 3 (the number, here "3", sets the planning scenario in planning_example_simulation.cpp)

3.2) For a real NAO:
  roslaunch rrt_connect_planner_example nao_extern.launch (for a real NAO, set IP and PORT for your NAO in nao_extern.launch)
  rosrun rrt_connect_planner_example planning_example_execution 3 (the number, here "3", sets the planning scenario in planning_example_execution.cpp)

//-------------------------------------------- EXECUTION -------------------------------------------------------
NAO starts execution immediatedly after successful planning (otherwise outputs an eror message if it failed to generated a plan 
for the given grasp pose or manipulation task)