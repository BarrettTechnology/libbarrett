#include <iostream>

extern "C" {
/* System Libraries */
#include <signal.h>
#include <sys/mman.h>

/* Package Dependencies */
#include <syslog.h>
//#include <libconfig.h>

/* Include the high-level WAM header file */

#include <libbarrett/gsl.h>
#include <libbarrett/wam.h>
//#include <libbarrett/wam_local.h>
}

/* Custom Dependencies */
#include "server_manager.h"

/* ------------------------------------------------------------------------ */

using namespace std;

/**server side manager which executes commands on the server command queue
runs as its own thread */

server_manager::server_manager(bt_wam * wam, 
                              queue<string> * pcmd_q, 
                              pthread_mutex_t * pmutex_q): 
                                 wam(wam), 
                                 pcmd_q(pcmd_q), 
                                 pmutex_q(pmutex_q), 
                                 state(IDLE)
{
   run(NULL);
}

server_manager::~server_manager()
{
}

void * server_manager::run(void *)
{
   string cmd_str(""); 
         
   while(1)
   {
      pthread_mutex_lock(pmutex_q);
      if (pcmd_q->size())
      {  
         cmd_str = pcmd_q->front();
         pcmd_q->pop();  
      }
      pthread_mutex_unlock(pmutex_q);

      if (cmd_str == "")         
         if (!execute(cmd_str) )
            cout << "invalid execution, unrecognized command" << endl;
           
      
    }
   return 0;
}

int server_manager::execute(string str)
{
  // double angles[7];
   
   if (str[0] == 'm')
   {
      return false; 
      /*
      get_angles(str.substr(1), 7, angles);
      bt_wam_moveto(wam, angles_to_vector(angles) ); */
   }   
   else if (str[0] == 'f') //force command
   {       
      return 1;      //IMPLEMENT
   }
   else if (str[0] == 't')
   {
       return 1;       //IMPLEMENT  
   } 
   
   else
      return -1;
      
   
   return 0;
}

bool get_angles(char * angle_str, int num_angles, double * d_array)
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
        if ( (*angle_str >= 46) & (*angle_str <= 57) | (*angle_str == '-') )           //if character ascii value is betw. period and 9
        {
            store[current++] = *angle_str;
            angle_str++;
        }
        else if (*angle_str == ',')   
        {
            angle_str++;
            store[current] = 0;
            current = 0;
            d_array[counter++] = atof(store);         //stores values 
        }
        
        else if (*angle_str == ' ')
              angle_str++;
        else
            return false;

    }
    d_array[counter++] = atof(store);         
    return (counter == num_angles);                   //checks if WAM DOF matches number of received values
}

gsl_vector * angles_to_vector(double * darray)
{
   
   
   
   return NULL;
}



