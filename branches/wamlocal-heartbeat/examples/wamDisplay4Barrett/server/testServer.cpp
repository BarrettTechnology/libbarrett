/*
 *      testServer.cpp
 *      
 *      Copyright 2009 robot <robot@robot-desktop>
 *      
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *      
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *      
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 *      MA 02110-1301, USA.
 */

#include "../sockets/sockets.h"
#include <iostream>

using namespace std;

static const int socket_port=2010;
static bool on = 0;

int main(int argc, char** argv)
{
   cout << "Initializing server..." << endl;
   Sockets serverSocket(socket_port); //, SOCKET_CONNECT_TIMEOUT_USEC);

   if (!serverSocket.error)
   {
      on = 1;
      cout << "server initialized" << endl;
   }
   
   char * out = "hello world!";
   
   while (on)
   {
      
      serverSocket.send(out, sizeof(out) );
      
   
      
      usleep(1000000);
         
   }
   
   
   return 0;
}
