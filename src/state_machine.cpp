#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/GotopointAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>


//  user is asked to start or not
bool start = false;

//  Reaching the robot to  goal or not
bool not_moving = true;

// calling the server

bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
	{
		if (req.command == "start")
			{
				start = true;
			}
		else 
			{
				start = false;
			}
		return true;
}


int main(int argc, char **argv)
{
	// Initialization of the node
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	
	// Initialization of the server

	ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
	actionlib::SimpleActionClient<rt2_assignment1::GotopointAction> ac("/Gotopoint");

	// min and max for the x and y values
	rt2_assignment1::RandomPosition rp; 
	rp.request.x_max = 5.0;
	rp.request.x_min = -5.0;
	rp.request.y_max = 5.0;
	rp.request.y_min = -5.0;
   
	while(ros::ok())
		{
			ros::spinOnce();
			// robot is instructed to move
			if (start)
				{
					// if the robot is not moving
					if (not_moving)
						{
							// calling for the new goal
							client_rp.call(rp);
							rt2_assignment1::GotopointGoal goal;
							goal.target_pose.header.frame_id = "base_link";
							goal.target_pose.header.stamp = ros::Time::now();
							goal.target_pose.pose.position.x = rp.response.x;
							goal.target_pose.pose.position.y = rp.response.y;
							goal.target_pose.pose.orientation.z = rp.response.theta;
							std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " <<rp.response.y << " theta = " <<rp.response.theta << std::endl;
							ac.sendGoal(goal);
							not_moving = false;
						}
					// if the robot moves
					else 
						{
							if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
								{
									std::cout << "Goal completed" << std::endl;
									not_moving= true;
								}
						}
				}
			// it is required to stop
			else 
				{
					// If it is already moving towards the goal
					if (!not_moving)
						{
							ac.cancelAllGoals();
							std::cout << "Goal cancelled" << std::endl;
							not_moving= true;
						}
				}
  

		}
   return 0;
}
