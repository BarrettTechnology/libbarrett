#include <iostream>

extern "C" {
/* System Libraries */
#include <signal.h>
#include <sys/mman.h>

/* Package Dependencies */
#include <syslog.h>
//#include <libconfig.h>
#include <pthread.h>

/* Include the high-level WAM header file */
#include <libbarrett/wam.h>
//#include <libbarrett/wam_local.h>
//#include <libbarrett/gsl.h>
//#include <libbarrett/wam_custom.h>
#include <libbarrett/os.h>
}

/* Custom Dependencies */
#include "../sockets/sockets.h"
#include "server_handler.h"



/* ------------------------------------------------------------------------ */

using namespace std;


/** sends WAM information to clients via socket */


server_handler::server_handler(Sockets * sock, 
                              bt_wam * wam, 
                              int * pconnected,
                              int * pgoing): sock(sock), 
                                             wam(wam), 
                                             pconnected(pconnected),
                                             pgoing(pgoing)
{
    run(NULL);

}

server_handler::~server_handler()
{

}

/*
int going;

void sigint_handler(int param)
{
   going = 0;
}
*/

void * server_handler::run(void *)
{
      char buf[100]; 
      int timeout = 0;
   
      while(*pconnected && *pgoing)
      {
         bt_wam_str_jposition(wam, buf);
//         cout << "sending str: " << line << ", of size " << strlen(line) << endl;
         int sending = sock->send(buf, sizeof(buf) );
         if (sending == -1)
         {
            cout << "error when sending to socket" << endl;
            timeout++;
            if (timeout > 10)
            {
               cout << "timeout exceeded, closing socket" << endl;
               //sock->socket_close();
               
               cout << "socket closed" << endl;
               break;
               //pthread_exit(0);
               //exit(1);
             }
          }
         
          bt_os_usleep(10000); //   slow loop to 100 Hz
       }   
   cout << "quiting handler" << endl;
   return 0;
}



