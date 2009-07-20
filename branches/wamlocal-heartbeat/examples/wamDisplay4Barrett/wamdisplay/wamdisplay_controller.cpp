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
#include "wamdisplay_controller.h"

/** requires btgw to be running on internal pc */


using namespace std;

//STATE state;

// TODO: may not need sockets

/*constructor */
wamdisplay_controller::wamdisplay_controller(double * shared_angle, 
										int shared_finish, 
										pthread_mutex_t * shared_mutex): shared_angle(shared_angle), 
																		shared_finish(shared_finish),
																		shared_mutex(shared_mutex)
{
}


/*destructor */
wamdisplay_controller::~wamdisplay_controller()
{
}

void * wamdisplay_controller::run(void )
{

    /* Stuff for starting up the WAM */
    struct bt_wam * wam;

    /* Lock memory??? */
    mlockall(MCL_CURRENT | MCL_FUTURE);

    /* Initialize syslog */
    openlog("demo", LOG_CONS | LOG_NDELAY, LOG_USER);

    /* Open the WAM (or WAMs!) */
    char address[100] = "tcp+json://192.168.139.150/wam7";
    
    cout << "trying to create wam" << endl;

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
	   CONTROL
	} state = VIEW;
    
	char buf[256];
   //char * line;
    while(going)
    {
#if 0
        switch(state)
        {
            case VIEW:
               pthread_mutex_lock( pmutex );
               bt_wam_str_jposition(wam, buf);
               if (!get_angles(buf, DOF,  darray ) )
               {
                  printf("failed in retrieving joint-positions");
                  exit(1);
               }
               pthread_mutex_unlock( pmutex );
               //       delete[] line;
               break;
               
            case CONTROL:
			
			
               break;

        }
#endif

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

bool wamdisplay_controller::get_angles(char * angle_str, int num_angles, double * d_array)
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

