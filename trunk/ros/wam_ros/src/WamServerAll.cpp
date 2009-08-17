/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... WamServer.cpp
 *  Author ............. vw
 *  Creation Date ...... 14 Aug 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  This program provides a WAM function call service
 *
 * ======================================================================== */


#include <iostream>
#include <string>

#include "ros/ros.h"
#include "wam_ros/WamCommands.h"
#include "wam_ros/WamState.h"
#include "../../../bindings/cpp/BtWam.hpp"



void getState(wam_ros::WamState * out, Wam * wam);

using namespace std;

char name[10] = "wam7";
Wam wam = Wam(name);

bool commands(wam_ros::WamCommands::Request  &req,
         wam_ros::WamCommands::Response &res )
{
	string str;
	char c = req.command;
	
	switch (c)
	{
		
		case 'g':
			wam.setGravityCompensation( (wam.isGravityCompensation())?0:1);
			ROS_INFO("gravity enabling");
			str = "gravity compensation changed";
			break;
		case 'h':
			if (wam.isHolding() )
			{
			   wam.idle();
			   ROS_INFO("idling");
			   str = "idle successful";
			}
			else
			{
			   wam.hold();
			   ROS_INFO("holding");
			   str = "holding successful";
			}							
			break;   
		case 'm':
			wam.moveHome();
			ROS_INFO("moving home");
			str = "moving home successful";
			break;
		case 'Y':
		 try{
			wam.teachStart();
			ROS_INFO("starting teach");
			str = "teach start successful";
			break;
		 }catch (WamException& e)
		 {
			cout << "Exception: " << e.what() << endl;
			exit(-1);
		 }
		case 'y':
			wam.teachEnd();
			ROS_INFO("ending teach");
			str = "end teach successful";
			break;
		case '.':
			wam.playback();
			ROS_INFO("playback now");
			str = "starting playback successful";
			break;
		default:
			break;



	}
		
	res.response = str;
	
	return true;
}

int main(int argc, char **argv)
{
	/* Initialize ros */
	ros::init(argc, argv, "wam_server");
  
  	/* Initialize WAM object */
	wam.init();
  
	/* Make node and publish wam_state */
	ros::NodeHandle n;
	ros::Publisher statePub = n.advertise<wam_ros::WamState>("wam_state", 1000);

	/* Advertise service for remote function calls */
	ros::ServiceServer service = n.advertiseService("commands", commands);

	/* Make loop run at 10 Hz */
	ros::Rate loop_rate(10);

	while (n.ok())
	{
		/* Stuff WamState message object and publish*/
		wam_ros::WamState out;
		getState(&out, &wam);
		statePub.publish(out);
	
		ROS_INFO_STREAM("publishing WAM state data " );
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


/* Stuffs WamState srv object with WAM state data */
void getState(wam_ros::WamState * out, Wam * wam)
{
	out->jpos = wam->getJointPosition();
	out->jvel = wam->getJointVelocity();
	out->jtor = wam->getJointTorque();

	out->cpos = wam->getCartesianPosition();
	out->crot[0] = wam->getCartesianRotationRow1();
	out->crot[1] = wam->getCartesianRotationRow2();
	out->crot[2] = wam->getCartesianRotationRow3();

	out->ctrl = wam->getCurrentControllerName();
	out->space = wam->getCurrentControllerSpace();
	out->pos = wam->conPosition();

	out->gcomp = wam->isGravityCompensation();
	out->holding = wam->isHolding();
	out->teaching = wam->isTeaching();
	out->loadedRefgen = wam->refgenLoadedName();
	out->activeRefgen = wam->refgenActiveName();
	out->moveIsDone = wam->moveIsDone();
}

