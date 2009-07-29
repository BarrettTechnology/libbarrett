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
         * NOTE: check CAN global variable below (line 71)
         
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
#include "ClientController.h"


#if DOF != 7
#define DOF 7
#endif

#define CAN 1  /// 1 if WAM hooked up externally through CAN. 0 if using internal PC server



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

void * initCliContrThd(void * );



void * initCliContrThd(void * _wamContrObj)
{
   ClientController * wamContrObj = (ClientController * ) _wamContrObj;
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
   char c;
   
   /* make threads */
   pthread_t thread1, thread2;
   
   std::cout << "ENTER if WAM is in folded position" << std::endl;
   getchar();
   
   std::cout << "ENTER if in IDLE mode" << std::endl;
   getchar();


#if 0
   std::cout << "Enter 1 if connected through CAN or enter 2 if connected through ethernet" << std::endl;
   c = getchar();

   if (c == '1')
   {
      std::cout << "Enter 1 if server is listening" << std::endl;
      
      std::cin >> c;
      CommunicationThread * t = new CommunicationThread(shared.angle, shared.finish, &mutex1);
      pthread_create( &thread2, NULL, initCommThd, t);
   }
   else if ( c == '2')
   {
      std::cout << "Enter 1 if bt-wam-gw is on" << std::endl;
      std::cin >> c;
      ClientController * wc = new ClientController(shared.angle, shared.finish, &mutex1);
      pthread_create( &thread2, NULL, initCliContrThd, wc);
   }
   else 
   {
      std::cout << "Invalid character. Aborting program." << std::endl;
      exit(1);
   }
#endif
   
   
   /* spin off graphics thread to maintain GUI */
   GraphicThread * g = new GraphicThread(shared.angle, shared.finish, &mutex1, (void *) &argument);
   pthread_create( &thread1, NULL, initGraThd, g);
   

#if CAN
   CommunicationThread * t = new CommunicationThread(shared.angle, shared.finish, &mutex1);
   pthread_create( &thread2, NULL, initCommThd, t);
#else
   ClientController * wc = new ClientController(shared.angle, shared.finish, &mutex1);
   pthread_create( &thread2, NULL, initCliContrThd, wc);
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
   void * threadResult = commThdObj->run();
   delete commThdObj;
   return threadResult;

}

void * initGraThd(void * _graThdObj)
{
   GraphicThread * graThdObj = (GraphicThread *) _graThdObj;
   //graThdObj = new GraphicThread(shared.angle, shared.finish, &mutex1, (void *) &argument);
   void * threadResult = graThdObj->run();
   delete graThdObj;
   return threadResult;
   
   
}
