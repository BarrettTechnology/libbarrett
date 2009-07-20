#include <string>
#include <iostream>
#include <queue>

extern "C" {
/* System Libraries */
#include <signal.h>
#include <sys/mman.h>
#include <pthread.h>

/* Package Dependencies */
#include <syslog.h>
//#include <libconfig.h>
}


/* Custom Dependencies */
#include "../sockets/sockets.h"
#include "server_controller.h"


/* ------------------------------------------------------------------------ */

/**server side controller which receives data from client WAM and places commands
on command queue */

using namespace std;

char line[50];

server_controller::server_controller(Sockets * sock, 
									pthread_mutex_t * pmutex_q, 
									queue<string> * pcmd_q, 
									int * pconnected, 
									int * pgoing) : sock(sock), pmutex_q(pmutex_q)
													pcmd_q(pcmd_q), pconnected(pconnected), pgoing(pgoing)
{   
   cout << "server controller initiated" << endl;
   
}


server_controller::~server_controller()
{

}

void * server_controller::run(void)
{
//    double angles[7];

   string line;
   cout << "entering server_controller" << endl;
    while(*pconnected && *pgoing)
    {
       cout << "dd" << endl;
       int read = sock->recv(&line, sizeof( line));
      if (read )
        cout << "printing out " << line << endl;
		//pthread_mutex_lock(*pmutex_q);
        //pcmd_q->push(line);
		//pthread_mutex_unlock(*pmutex_q);
	
      else if (read == -1)
         printf("error reading");
      usleep(2000);
       
/*
        sock.recv(line, sizeof(line) );
        if (!get_angles(line) , DOF, &angles)
            printf("failed in retrieving joint-positions");
*/
    //    pthread_mutex_lock(&mutex_q );
/*        for (unsigned int i = 0; i < DOF; i++)
            server_values.angles[i] = angles[i];*/
        
        
           
   //     pthread_mutex_unlock(&mutex_q);


    }
   
   return 0;
}



