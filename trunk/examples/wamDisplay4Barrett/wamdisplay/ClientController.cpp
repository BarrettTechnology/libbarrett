#include <iostream>

extern "C" {
/** C libraries */
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/mman.h>               //for mlockall()
#include <syslog.h>                 //for logging

/* Include the high-level WAM header file */
#include <libbarrett/wam.h>
}
#include "../sockets/sockets.h"
#include "wamComponents.h"
#include "ClientController.h"

//! requires bt-wam-gw to be running on internal pc */


using namespace std;

//STATE state;

/* TODO: add CONTROL state which has listens for GUI actions and 
 * translates into WAM movement
 */

/*constructor */
ClientController::ClientController(double * shared_angle, 
                              int shared_finish, 
                              pthread_mutex_t * shared_mutex): shared_angle(shared_angle), 
                                                      shared_finish(shared_finish),
                                                      shared_mutex(shared_mutex)
{
}


/*destructor */
ClientController::~ClientController()
{
}

void * ClientController::run(void )
{

    /* Stuff for starting up the WAM */
    struct bt_wam * wam;

    /* Lock memory??? */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Initialize syslog */
    openlog("demo", LOG_CONS | LOG_NDELAY, LOG_USER);

    /* Open the WAM (or WAMs!) */ //! address will vary
    char address[100] = "tcp+json://wam15/wam7";//"tcp+json://192.168.139.150/wam7";
    

    wam = bt_wam_create(address);           //how to pass in argv[1]???
    if (!wam)
    {
        closelog();
        printf("Could not open the WAM.\n");
        return 0;                                                           //return -1???
    }

   cout << "wam created" << endl;
    /* Loop until Ctrl-C is pressed ???*/
    int going = 1;
   enum  
   {
      VIEW,
      //CONTROL
   } state = VIEW;
    
   char buf[256];
   //char * line;
    while(going)
    {

        switch(state)
        {
            case VIEW:
               pthread_mutex_lock( shared_mutex );
               bt_wam_str_jposition(wam, buf);
               if (!get_angles(buf, DOF,  shared_angle ) )
               {
                  printf("failed in retrieving joint-positions");
                  exit(1);
               }
               pthread_mutex_unlock( shared_mutex );
               //       delete[] line;
               break;
            
            default:
         
         
               break;

        }


       /* ?????? Slow this loop down to about 10Hz ?????*/
       usleep(100000); /* Wait a moment*/

    }

    /* Close the WAM */
    bt_wam_destroy(wam);

    /* Close syslog */
    closelog();


return 0;
}


/**NOTE: joint positions are called angles here??? */

/**parses string of angles from WAM and stores them in shared variable
param: angle_str- char * of streamed string
param: num_angles- DOF of wam, does not have to be 7 but is max value
param: sharedvar- 
retval: true- correct parsing; false- incorrect parsing and/or input string

*/

bool ClientController::get_angles(char * angle_str, int num_angles, double * d_array)
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

#if 0
void toggle_state(char c)
{
   if (c == 'v')
      state = VIEW;
   else if (c == 'c')
      state = CONTROL;
   else
      return;
   
}

#endif

