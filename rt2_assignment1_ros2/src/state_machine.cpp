
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1_ros2/srv/command.hpp"
#include "rt2_assignment1_ros2/srv/position.hpp"
#include "rt2_assignment1_ros2/srv/random_position.hpp"
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
using namespace std;
using UserInt = rt2_assignment1_ros2::srv::Command;
using GoalPos = rt2_assignment1_ros2::srv::Position;
using RndPos = rt2_assignment1_ros2::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
bool start = false;
bool doneOnce= false;
namespace rt2_assignment1_ros2 {
	

class GoalPositionClient : public rclcpp::Node
	{
		public:
			GoalPositionClient() : Node("Goal_client")
				{
						// initialization of the client: calls the server '/go_to_point'
						client_ = this->create_client<GoalPos>("/go_to_point");
						while(!client_ -> wait_for_service(std::chrono::seconds(1)))
							{
								if (!rclcpp::ok())
									{
										RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
										return;
									}
								//RCLCPP_INFO(this->get_logger()), "aiting for service /go_to_point to appear.");
							}
						this->request_=std::make_shared<GoalPos::Request>();
						this->response_=std::make_shared<GoalPos::Response>();	
				}
				
				void call_serverGTP()
					{
						auto result_future= client_->async_send_request(request_);
						if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) != rclcpp::FutureReturnCode::SUCCESS)
							{
								RCLCPP_ERROR(this->get_logger(), "service call failed");
							}
						this->response_=result_future.get();
					}
					
				std::shared_ptr<GoalPos::Request> request_;
				std::shared_ptr<GoalPos::Response> response_;
	
		private:
			rclcpp::Client<GoalPos>::SharedPtr client_;
	
	};	
	
class StateMachine : public rclcpp::Node
	{
		public:
			StateMachine(const rclcpp::NodeOptions & options) : Node("State_machine", options)
				{
					
					// the function timer_callback: executed every 500 ms
					timer_ =this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&StateMachine::timer_callback, this));
					
					// server '/user_interface' initialized
					service_ = this->create_service<UserInt>("/user_interface", std::bind(&StateMachine::handle_service, this, _1, _2, _3));
					
					// client initialized for the node 'position_server'
					RP_client=this->create_client<RndPos>("/position_server");
					while(!RP_client -> wait_for_service(std::chrono::seconds(10)))
						{
							if (!rclcpp::ok())
								{
									RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
									return;
								}
						}
				}

		private:

			// function called as the client request this server
			void handle_service( 
			const std::shared_ptr<rmw_request_id_t> request_header,
			const std::shared_ptr<UserInt::Request> request,
			const std::shared_ptr<UserInt::Response> response)
			{
				(void) request_header;
				// if the request command is 'stop', then the global variable 'start' is false
				if (request->command == "stop")
					{
						start= false;
						response->ok = true;
							
					}
				// if the request command is 'start', then it is true
				else if ( request->command == "start")
					{
						start= true;
						response->ok=true;	
					}
				response->ok=true;
			}
			
			void timer_callback()
				{
					//auto goal_position = std::make_shared<GoalPositionClient>();
					auto random_position = std::make_shared<RndPos::Request>();
					// executing the timer_callback right after one request was served, nothing happens
					if (doneOnce)
						{
							doneOnce=false;
						}
					else if (start == false)
						{
							// Nothing happens
						}
					else if ( start == true)
						{
							// the position_server is called to get a new random position within the max and min range
							random_position->x_max = 5.0;
							random_position->x_min = -5.0;
							random_position->y_max = 5.0;
							random_position->y_min = -5.0;
							using ServiceResponseFuture =rclcpp::Client<RndPos>::SharedFuture;
							auto response_received_callback= [this] (ServiceResponseFuture future) 
								{
									auto goal_position = std::make_shared<GoalPositionClient>();
									goal_position->request_->x=future.get()->x;
									goal_position->request_->y=future.get()->y;
									goal_position->request_->theta=future.get()->theta;
																std::cout << "\nGoing to the position: x= " << goal_position->request_->x << " y= " <<goal_position->request_->y << " theta = " <<goal_position->request_->theta << std::endl;
							goal_position->call_serverGTP();
							std::cout << "Position reached" << std::endl;
								};
							auto future_result= RP_client->async_send_request(random_position, response_received_callback);
							// when the 'go_to_point' server is called: the robot moves in the goal position x and y
							/*goal_position->request_->x=Xg;
							goal_position->request_->y=Yg;
							goal_position->request_->theta=Tg;
							std::cout << "\nGoing to the position: x= " << goal_position->request_->x << " y= " <<goal_position->request_->y << " theta = " <<goal_position->request_->theta << std::endl;
							goal_position->call_serverGTP();
							std::cout << "Position reached" << std::endl;*/
							doneOnce=true;
						}		
				}
			rclcpp::Service<UserInt>::SharedPtr service_;
			rclcpp::TimerBase::SharedPtr timer_;
			rclcpp::Client<RndPos>::SharedPtr RP_client;
	};
	
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_ros2::StateMachine)	

