#include <iostream>

#define GL_GLEXT_PROTOTYPES

extern "C" {
#include <math.h>
#include <stdlib.h>
#include <stdio.h>   
#include <string.h>
#include <GL/glut.h>
#include <GL/gl.h>
#include <unistd.h>
#include <stdarg.h>
#include <pthread.h>
#include "wamComponents.h"
#include "dhTransform.h"
#include "../sockets/sockets.h"
#include "wamdisplay_controller.h"
#include "../include/definitions.h"
#include "../include/wam_Spec.h"
}

#include "client_handler.h"   

#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
   fflush(stdout);

   int timeout = 0;
   bool communicate = false;   
   
   char ip[20] = "localhost";
   int socket_port = 2021;
   
   /* create client socket */
   cout << "trying to connect to server" << endl;
   Sockets clientSocket(ip, socket_port);
   
   if(!clientSocket.error){ //if connection successful
      communicate=true;
      cout << "connection to WAM server successful" << endl;
   }
   
   char buf[50];
   client_handler::rand_num_gen(8, buf);
   cout << buf << endl;
   
   while(communicate)
   {
      client_handler::rand_num_gen(8, buf);
      
//         cout << "sending str: " << line << ", of size " << strlen(line) << endl;
      int sending = clientSocket.send(buf, sizeof(buf) );
      
      if (sending == -1)
      {
         cout << "error when sending to socket" << endl;
         timeout++;
         if (timeout > 10)
         {
            cout << "timeout exceeded, closing socket" << endl;
            //sock->socket_close();
            
            cout << "socket closed" << endl;
            //break;
            //pthread_exit(0);
            exit(1);
          }
       }
      
      usleep(10000); //   slow loop to 100 Hz
       
    }
  
   return 0;
}

