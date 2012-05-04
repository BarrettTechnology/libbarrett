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
#include <cstdlib>
#include <string.h>
#include <assert.h>

#include "ros/ros.h"
#include "wam_ros/WamCommands.h"
#include "../../../bindings/cpp/BtWam.hpp"

using namespace std;

double * moveInterface();


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
	ros::ServiceClient moveClient;	

	client = n.serviceClient<wam_ros::WamCommands>("commands");
	moveClient = n.serviceClient<wam_ros::WamCommands>("move_cmd");
	
	wam_ros::WamCommands srv;
	
	cout << "client init'ed" << endl;
	
	while (c != 'x')
	{
		

		cout << "input a command: ";

		cin >> c;  //cin cannot handle spaces
		cout << "input is: " << c << endl;
		
		if (c == 't')
		{
			double * d;
			d = moveInterface();
			for (int i = 0; i < 7; i++)
				srv.request.desiredJoints[i] = d[i];
			//srv.request.desiredJoints = d;
			
			if (moveClient.call(srv))
			{
				ROS_INFO("success!");//, srv.response.response);
			}
			else
			{
				ROS_ERROR("Failed to call service");
				return 1;
			}
		}
		else
		{			
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




		

	
	}
	
	


	return 0;
}


double * moveInterface()
{
	double pos[7];
	string str;
	int i = 0;
	char cstr[50];
	char * pch;

	
	cout << "Please enter 7 values separated by spaces." << endl;
	
	getline(cin, str);
	
	cout << "you input " << str << endl;
	
	strcpy(cstr, str.c_str());
		
	pch = strtok (cstr," ");
	
	cout << cstr << endl;
	
	while ( (pch != NULL) & (i < 7 ))
	{
		pos[i++] = atof(pch);
		pch = strtok (NULL, " ");
	}
	
	cout << "i is after " << i << endl;
	
	for ( i = i; i < 7; i++)
		pos[i] = 0;
	
	cout << "printing double array" << endl;
	for (int j = 0; j < 7; j++)
		cout << pos[j] << endl;
		
	cout << "i is " << i << endl;
	assert(i == 7);
	return pos;
}

