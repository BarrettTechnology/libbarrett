/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... WamJpos.cpp
 *  Author ............. vw
 *  Creation Date ...... 30 July 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  This program demonstrates simple sending of messages over the ROS system 
 *  integrated withlibbt. Requires WamServer to be running.
 *
 * ======================================================================== */


#include "ros/ros.h"
#include "ros/master.h"
#include "std_msgs/String.h"
#include "wam_ros/WamState.h"
#include "wam_ros/Joints.h"
#include "../../../bindings/cpp/Posix.hpp"
#include "../../../bindings/cpp/Mutex.hpp"

#include <string>

#include <iostream>
#include <curses.h>
#include <signal.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

using namespace std; 

/* function prototypes*/
void getWamState(const wam_ros::WamState::ConstPtr& msg);
void * display(void * _obj);

/* global shared wam state variable structure*/
struct WamData
{
	int status; 		//! 0 = can't find master. 1 = can't find publisher node. 2 = successful subscription
	string jpos;
	string jvel;
	string jtor;

	string cpos;
	string crot1;
	string crot2;
	string crot3;

	string ctrl;
	string space;
	string pos;

	int gcomp;
	int holding;
	int teaching;
	string loadedRefgen;
	string activeRefgen;
	int moveIsDone;
} wamdata;

/* global mutex */
Mutex mutex = Mutex();

/* We have a global flag and signal handler
 * to allow the user to close the program
 * with [Control+C] */
int going;
void sigint_handler(int param)
{
   going = 0;
}

int main(int argc, char **argv)
{
   wamdata.status =0;

   Posix thd(display, NULL);
   thd.start();

   /* Initializes ROS system. wam_listener is name of node */
   ros::init(argc, argv, "wam_listener");

   /* NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.   */
   ros::NodeHandle n;



   /* Subscribe to wam_jpos topic. Data passed to chatterCallBack function.   */
   ros::Subscriber sub = n.subscribe("wam_state", 1000, getWamState);

   /* Set loop frequency to 10Hz */
   ros::Rate loop_rate(10);
	
   while (n.ok())
   { 
	/* To keep subscribing */
	ros::spinOnce();

	/* Sleep to ensure correct loop time */
	loop_rate.sleep();
   }

//  thd.stop();  /* not needed? weird. must be declared twice */
   endwin();
    
   return 0;
}

/* Reads and stores incoming data from WAM state */
void getWamState(const wam_ros::WamState::ConstPtr & msg)
{
	mutex.lock();

	wamdata.jpos = msg->jpos;
	wamdata.jvel = msg->jvel; 
	wamdata.jtor = msg->jtor;

	wamdata.cpos = msg->cpos;
	wamdata.crot1 = msg->crot[0];
	wamdata.crot2 = msg->crot[1];
	wamdata.crot3 = msg->crot[2];

	wamdata.ctrl =  msg->ctrl;
	wamdata.space = msg->space;
	wamdata.pos = msg->pos;

	wamdata.gcomp = msg->gcomp ;
	wamdata.holding = msg->holding;
	wamdata.teaching = msg->teaching;
	wamdata.loadedRefgen = msg->loadedRefgen;
	wamdata.activeRefgen = msg->activeRefgen;
	wamdata.moveIsDone = msg->moveIsDone;

	mutex.unlock();

}

/* Text GUI, looks like bt-wam-demo */
void * display(void * _obj)
{

   
   /* Initialize ncurses */
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();


   enum {
      SCREEN_MAIN,
      SCREEN_HELP
   } screen;
   
     screen = SCREEN_MAIN;
     
   /* Register the ctrl-c interrupt handler
    * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
   going = 1;

   while (going)
   {

      /* Clear the screen buffer */
      clear();
      
      /* Display the display or help screen */
      switch (screen)
      {
         int line;
         case SCREEN_MAIN:
	    mutex.lock();

            line = 0;
            
            /* Show HEADER */
            mvprintw(line++, 0, "Barrett Technology - Demo Application\t\tPress 'h' for help");
            line++;

            /* Show controller name (joint space, cartesian space) */
            mvprintw(line++, 0, " Controller: %s",  wamdata.ctrl.c_str());

            /* Show GRAVTIY COMPENSATION */
            mvprintw(line++, 0, "GravityComp: %s", wamdata.gcomp ? "On" : "Off" );
            
            /* Show HOLDING */
            mvprintw(line++, 0, "    Holding: %s", wamdata.holding ? "On" : "Off" );
			mvprintw(line++, 0, "   Teaching: %s", wamdata.teaching? "On" : "Off" );
            
            mvprintw(line+0, 25, " Loaded Refgen: %s", wamdata.loadedRefgen.c_str() );
            mvprintw(line+1, 25, " Active Refgen: %s", wamdata.activeRefgen.c_str() );
            
            mvprintw(line++, 0, " MoveIsDone: %s", wamdata.moveIsDone ? "Done" : "Moving" );
            

            line++;
            
#if 0
            /* Show trimesh refgen stuff */
            if (tri)
            {
               mvprintw(line++, 0, "     pos: %s", bt_gsl_vector_sprintf(buf,tri->pos) );
               mvprintw(line++, 0, "      hs: %s", bt_gsl_vector_sprintf(buf,tri->hs) );
               line++;
            }
#endif        
            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", wamdata.jpos.c_str() );
            mvprintw(line++, 0, "J Velocity : %s", wamdata.jvel.c_str() );
            mvprintw(line++, 0, "J Torque   : %s", wamdata.jtor.c_str() );
            line++;

            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, "C Position : %s", wamdata.cpos.c_str());
            
            mvprintw(line,   0, "C Rotation :");
            mvprintw(line++, 13, "%s", wamdata.crot1.c_str() );
            mvprintw(line++, 13, "%s", wamdata.crot2.c_str() );
            mvprintw(line++, 13, "%s", wamdata.crot3.c_str() );
            line++;
			
	    mutex.unlock();
			
            break;
			
         case SCREEN_HELP:
            line = 0;
            mvprintw(line++, 0, "Help Screen - (press 'h' to toggle)");
            line++;
            mvprintw(line++, 0, "g - toggle gravity compensation");
            break;
			
			
      }
      
      /* Display the screen */
      refresh();
      
      usleep(100000); //10hz
   
   }
       
   /* Close ncurses, not needed? called in main function */
   endwin();
  
   return 0;
}
