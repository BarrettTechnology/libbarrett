/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... WamClient.cpp
 *  Author ............. vw
 *  Creation Date ...... 14 Au 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  This program sends remote function calls to WamServer. Requires
 *  WamServer to be running.
 *
 * ======================================================================== */


#include <iostream>
#include <string>
#include <cstdlib>


#include "ros/ros.h"
#include "wam_ros/WamCommands.h"
#include "../../../bindings/cpp/BtWam.hpp"

using namespace std;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wam_client");

#if 0
	if (argc != 3)
	{
	ROS_INFO("usage: add_two_ints_client X Y");
	return 1;
	}
#endif

	ros::NodeHandle n;
	char c;
	
	cout << "init client" << endl;
	
	ros::ServiceClient client;
	client = n.serviceClient<wam_ros::WamCommands>("commands");
	
	wam_ros::WamCommands srv;
	
	cout << "client init'ed" << endl;
	
	while (c != 'x')
	{
		

		cout << "input a command: ";

		cin >> c;  //cin cannot handle spaces
		cout << "input is: " << c << endl;
		
		/* package character as uint64 to send to service via ROS */
		srv.request.command = c;
		
		cout << "sending command is " << srv.request.command << endl;

		if (client.call(srv))
		{
			ROS_INFO("success!");//, srv.response.response);
		}
		else
		{
			ROS_ERROR("Failed to call service");
			return 1;
		}


		

	
	}
	
	


	return 0;
}
