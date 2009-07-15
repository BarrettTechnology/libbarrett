/***************************************************************************
 *   Copyright (C) 2007 by Pedro Nobre                                     *
 *   pedrognobre@gmail.com                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*! \file wamdisplay.cpp
    \author Pedro Nobre <pedrognobre@gmail.com>
    \author Pedro Queirós <pqueiros@isr.uc.pt>
    \author Cristóvão Sousa <crisjss@isr.uc.pt>
	
	\Revised: 13 July 2009- 
			Victor J Wang, Barrett Technology Inc.
			* Moved communication and graphic threads into separate classes and files

    \mainpage

The main code ( wamdisplay.cpp )is compose by 2 threads. One of them is responsible to deal with openGL routines. The other one preforms the communications with other programs using sockets to receive WAM arm angles. Each socket carries a table with 7 floats:
 -# tab[0]=theta1;
 -# tab[1]=theta2;
 -# tab[2]=theta3;
 -# tab[3]=theta4;
 -# tab[4]=theta5;
 -# tab[5]=theta6;
 -# tab[6]=theta7;

This architecture was adopted to make the display  refresh rate independent from communication data rate.
Each time new data arrives, a shred memory struct is replaced. If the data arrives very slowly some values are display more than once. Otherwise, if it cames to fast some values are just ignored.<BR>
This program wam inplemented as a server and should be the first to be started.
*/



extern "C" {
#include <math.h>
#include <stdlib.h>
#include <stdio.h>   
#include <string.h>
#include <unistd.h>
#include <pthread.h>
}

#include <iostream>

#include "CommunicationThread.h"

#include "wamdisplayGlobals.h"
#include "GraphicThread.h"
#include "wamdisplay_controller.h"


#if DOF != 7
#define DOF 7
#endif

#define CAN 0  /// 1 if WAM hooked up externally through CAN. 0 if using internal PC server



static const int socket_port=2021; ///<socket listening port

//ammount of time that the communication thread will be blocked waiting for a new connection
static const long socket_connect_timeout_usec=10*1e6; //-1 to not use timeout
static const long socket_recv_timeout_usec=1000*1e3;  //-1 to not use timeout

pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;


//shared memory values
/*
struct shared_values{
  double angle[7];
  int finish;
  //pthread_mutex_t mutex1;
};
shared_values shared; 

struct arguments
{
	int argc0;
	char **argv0;
}argument;
*/



//threads
//void *graphicThread(void *);
//void * communicationThread(void *);

void * initCommThd(void * );
void * initGraThd(void * );

void * initWamContrThd(void * );



void * initWamContrThd(void * _wamContrObj)
{
   printf("init wamdisplay_controller");
   wamdisplay_controller * wamContrObj = (wamdisplay_controller * ) _wamContrObj;
   void * threadResult = wamContrObj->run();
   delete wamContrObj;
   return threadResult;
}





/* global variables make static?? */
static arguments argument; 	
static shared_values shared;

int main(int argc, char *argv[])
{
  
	/* set joint positions to 0 */
	for(int i=0;i<7;i++)
		shared.angle[i]=0;
	shared.finish=1;
  
	/* initialize global variable */
	argument.argc0=argc;
	argument.argv0=argv;

	/* make threads */
	pthread_t thread1, thread2;
	
	/* spin off graphics thread to maintain GUI */
	GraphicThread * g = new GraphicThread(shared.angle, shared.finish, &mutex1, (void *) &argument);
	pthread_create( &thread1, NULL, initGraThd, g);
	
	
#if CAN
	CommunicationThread * t = new CommunicationThread(shared.angle, shared.fi nish, &mutex1);
	pthread_create( &thread2, NULL, initCommThd, t);
#else
	wamdisplay_controller * wc = new wamdisplay_controller(shared.angle, shared.finish, &mutex1);
	pthread_create( &thread2, NULL, initWamContrThd, wc);
#endif 

	pthread_join( thread1, NULL);
	pthread_join( thread2, NULL);
	exit(0);
	return 0;
}


void * initCommThd(void * _commThdObj)
{
	CommunicationThread * commThdObj = (CommunicationThread *) _commThdObj;
	//commThdObj = new CommunicationThread(shared.angle, shared.finish, &mutex1);
	printf("starting comm thread");
	void * threadResult = commThdObj->run();
	delete commThdObj;
	return threadResult;

}

void * initGraThd(void * _graThdObj)
{
	GraphicThread * graThdObj = (GraphicThread *) _graThdObj;
	//graThdObj = new GraphicThread(shared.angle, shared.finish, &mutex1, (void *) &argument);
	printf("starting graphics thread");
	void * threadResult = graThdObj->run();
	delete graThdObj;
	return threadResult;
	
	
}
