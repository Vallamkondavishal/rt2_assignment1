/** @ package rt2_assignment1
* 
*  \file state_machine.cpp
*  \brief This file implements the finite state machine behaviour
*
*  \author vishal vallamkonda
*  \version 1.0
*  \date 28/02/2022
*  \details
*   
*  Subscribes to: <BR>
*	 None
*
*  Publishes to: <BR>
*	 /reach
     /time
*
*  Services: <BR>
*    /user_interface
* 
*  Client: <BR>
	 /position_server
*
*  Action Client: <BR>
*    /go_to_point
*
*  Description: <BR>
*    This node is a server for the user interface, it receives what the user
* 	 writes and it acts depending on it. If the client ask for the random 
* 	 position behaviour it calls the server /random_position and it waits 
*  	 for it to be finished, it also checks if the client requests the behaviour
* 	 to stop and in that case it cancels the previous goal and waits for the
* 	 next command. In the mean time it also sends information to the user_interface
* 	 node, it publishes a topic when a goal has been reached (True) or cancelled
* 	 (False) so that the user_interface can keep track of the reached and cancelled
* 	 goals. Also whenevera goal is reached it publishes on the topic /time the
* 	 time in seconds between the request of the goal and the completion of it.
*/

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/RandomPosition.h"
#include <rt2_assignment1/GotopointAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

bool start = false;  /* needed to know if the user asked to stop or not */

bool not_moving = true;	/* used to know wheather robot is already moving towards a goal or not*/

/**
 * \brief: It receives the commands from the user_interface node
 * \param req: the command received from the client
 * \param res: not set
 * 
 * \return: true
 * 
 * This function sets the global variable start to true if the command 
 * received is "start" or it sets it to false  if the command is different
 */


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

/**
 * \brief: The main function
 * 
 * \return: 0
 * 
 * This function initializes the ros node, the server, the clients, the publishers.
 * Then if ros is running it checks the global variable start. If start 
 * is true it is also checked if the robot is already moving towards a goal 
 * with the global variable notgoing. If the robot is not already moving
 * a new goal is set, if on the other hand the robot was already going it 
 * is checked if the goal is reached, in this case information are published
 * on the topics /time and /reach. If the global variable start is false 
 * I check if the robot is already moving, if that is the case I cancel 
 * the goal and publish information on the /reach topic. If the robot is 
 * still and the start variable is false the program does nothing.
 */

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_machine");
	ros::NodeHandle n;
	ros::NodeHandle n1;
	
	ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
	ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
	actionlib::SimpleActionClient<rt2_assignment1::GotopointAction> ac("/Gotopoint");
   
	rt2_assignment1::RandomPosition rp; 
	rp.request.x_max = 5.0;
	rp.request.x_min = -5.0;
	rp.request.y_max = 5.0;
	rp.request.y_min = -5.0;
   
	while(ros::ok())
		{
			ros::spinOnce();
			if (start)
				{
					if (not_moving)
						{
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
					else 
						{
							if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
								{
									std::cout << "Goal completed" << std::endl;
									not_moving= true;
								}
						}
				}
			else 
				{
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
