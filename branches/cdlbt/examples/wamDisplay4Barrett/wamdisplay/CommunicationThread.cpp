/* ======================================================================== *
 *  Module ............. wamdisplay
 *  File ............... communicationThread.cpp
 *  Author ............. Victor J Wang
 *  Creation Date ...... 13 July 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  NOTES:
 *
 *  REVISION HISTORY:
 * 
 *                                                                          *
 * ======================================================================== */
/** Communication thread for WAM GUI. Receives WAM position streams via socket 
 * and updates GUI in real-time. 
 * */

extern "C" {
#include <math.h>
#include <stdlib.h>
#include <stdio.h>   
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>

#include "../sockets/sockets.h"
//#include "wamdisplay_controller.h"
//#include "client_handler.h"
#include "wamdisplayGlobals.h"
}




#include <iostream>
#include "CommunicationThread.h"

#if DOF != 7
#define DOF 7
#endif

#define SIMPLE 0                     //0 to support only VIEW mode. 1 to support multiple modes **NOT IMPLEMENTED**

static const int socket_port=2021; ///<socket listening port

using namespace std;

#if 0
extern double angle[7];
extern int finish;


#endif
extern pthread_mutex_t mutex1;

bool parse_angles(char *, int, double *);

/* Constructor */
CommunicationThread::CommunicationThread(double * shared_angle, 
                              int shared_finish, 
                              pthread_mutex_t * shared_mutex): shared_angle(shared_angle), 
                                                      shared_finish(shared_finish),
                                                      shared_mutex(shared_mutex)
{
}


/* Destructor */
CommunicationThread::~CommunicationThread()
{
}


/** Main loop **/
void * CommunicationThread::run(void)
{
   double socket_angles[7];
   char ip[20] = "localhost";   
   bool communicate;
   char in;
   int loop = 1;
   
   enum {
      VIEW,
      
#if SIMPLE
      MOVETO,
      REFGEN,
      FORCE
#endif
   } state = VIEW;
   

   while (shared_finish)
   {
      fflush(stdout);

      int timeout = 0;
      
      communicate=false;
     char string[100];
      
      /* create client socket */
      std::cout << "trying to connect to server" << std::endl;
      Sockets clientSocket(ip, socket_port);

      if(!clientSocket.error){ //if connection successful
         communicate=true;
         std::cout << "connection to WAM server successful" << std::endl;
      }

      while(communicate)
      {  
        switch(state)
        {
         case VIEW:                        
            /*try and read from socket */
            int received = clientSocket.recv(string, sizeof(string) );

            if (received == -1 )
            std::cout << "error occured when reading from socket" << std::endl;
            
            /* if no message received for 10 rounds, assumes server dropped*/
            else if (!received)
            {
               timeout++;
               std::cout << "empty message" << std::endl;
               if (timeout > 10)
               {
                  std::cout << "server no longer detected" << std::endl;
                  communicate = 0;
                  break;
               }
            }
            else
            {

               /*parse received string and store in local variable */
               parse_angles(string, 7, socket_angles);

               /*update shared variables for GUI */
               pthread_mutex_lock( shared_mutex );
               for(int i=0;i<7;i++)
                  shared_angle[i]=socket_angles[i];
               pthread_mutex_unlock( shared_mutex ); 
               usleep(10000);      //slow loop to 100 Hz
            }
            break;
#if SIMPLE      
         case MOVETO:
            std::cout << "entering moveto state" << endl;
            client_handler::angles_to_string(string, * double_array);
            clientSocket.send(string, sizeof(string) );
            break;
            
         case REFGEN:
         
            break;
            
            
         case FORCE:
         
            break;
            
#endif
        }
        
  
      }
      
      /* asks user to attempt reconnection */
      while(loop)
      {
         std::cout << "retry connection?(y/n)" << std::endl;
         std::cin >> in;
         if ( (in == 'y') | (in == 'Y') )
            break;
         else if ( (in == 'n') | (in == 'N') )
         {
            //return 0;
            exit(0);
         }
         else 
            std::cout << "unrecognized character, please try again" << std::endl;
            
      }
      
      } 
   return 0;
}




/**NOTE: joint positions are called angles here */

/**parses string of angles from WAM and stores them in shared variable
param: angle_str- char * of streamed string
param: num_angles- DOF of wam, does not have to be 7 but is max value
param: sharedvar- 
retval: true- correct parsing; false- incorrect parsing and/or input string

*/

bool parse_angles(char * angle_str, int num_angles, double * d_array)
{
    char store[20] = "";
    int current = 0;        //index pointer to string

    /* to keep track of which angle */
    int counter = 0;

    /*check if starting char is correct */
    if (*angle_str != '<')
        return false;

    angle_str += 2;

    /*begin parsing  */
    while(*angle_str != '>')
    {
        //if character ascii value is betw. period and 9
        if ( ( (*angle_str >= 46) & (*angle_str <= 57) ) | (*angle_str == '-') ) 
        {
            store[current++] = *angle_str;
            angle_str++;
        }
        //if comma, reset
        else if (*angle_str == ',')   
        {
            angle_str++;
            store[current] = 0;
            current = 0;
            d_array[counter++] = atof(store);         //stores values 
        }
        
        //skip if empty space
        else if (*angle_str == ' ')
              angle_str++;
        else
            return false;

    }
    d_array[counter++] = atof(store);         
    return (counter == num_angles);                   //checks if WAM DOF matches number of received values
}











