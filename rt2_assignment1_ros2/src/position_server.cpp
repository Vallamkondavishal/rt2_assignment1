
#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1_ros2/srv/random_position.hpp"
#include <iostream>

using namespace std;
using RndPos = rt2_assignment1_ros2::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1_ros2 {
	
	class random : public rclcpp :: Node
		{
			public : 
				random (const rclcpp::NodeOptions & options) : Node("position", options)
					{
						// initialization of the server /position_server
						server_ = this->create_service<RndPos>("/position_server", std::bind(&random::handle_service, this, _1, _2, _3) );
					}

			
			private : 
				// function that returns a random number between M and N
				double randMToN(double M, double N)
					{     
						return M + (rand() / ( RAND_MAX / (N-M) ) ) ; 
					}
			
				// function executed when there is a request from the client
				void handle_service(
					const std::shared_ptr<rmw_request_id_t> request_header,
					const std::shared_ptr<RndPos::Request> request,
					const std::shared_ptr<RndPos::Response> response)
					{
						(void)request_header;
						// I send as a response a random number in the given interval
						response->x= randMToN(request->x_min, request->x_max);
						response->y= randMToN(request->y_min, request->y_max);
						response->theta = randMToN(-3.14, 3.14);
					}
					
					rclcpp::Service<RndPos>::SharedPtr server_;
			
		};
	




}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_ros2::random)
