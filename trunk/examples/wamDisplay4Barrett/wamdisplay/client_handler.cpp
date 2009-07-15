#include <iostream>
#include <string>
#include <sstream>

extern "C" 
{
#include <stdio.h>
#include <stdlib.h>
#include <libbarrett/wam.h>

}

/* Custom Dependencies */
#include "../sockets/sockets.h"

#include "client_handler.h"


/* ------------------------------------------------------------------------ */

/** Client handler class 
 * static class called by client GUI to send information to WAM server 
 * for movement updates */

using namespace std;

client_handler::client_handler(Sockets * sock): sock(sock)
{
}

client_handler::~client_handler()
{
}

/*
void client_handler::move_wam_to_point()
{
   
   
   
}
*/

char *  client_handler::rand_num_gen(int size, char * str)
{
	srand(time(NULL));			//to ensure actual random generation
   str[0] = '\n';
   strcat(str, "< ");				// '\n' is overwritten by first character in concatenate
   char buf[10];
   
   for (int i = 0; i < size-1; i++)
   {
      sprintf(buf, "%d", i);      
      strcat(str, buf );
      strcat(str, ".");
      sprintf(buf, "%d", (rand() %10) + 1 );
      strcat(str, buf );
      strcat(str, ", ");
   }
   sprintf(buf, "%d", size-1);      
   strcat(str, buf );
   strcat(str, " >");
   
   return str;
}

char * client_handler::angles_to_string(char * str, double * array)
{
	str[0] = '\n';							//to ensure proper concatenation
	char buf[10];
	
	int size = sizeof(array)/sizeof(double);
	
	for (int i = 0; i < size-1; i++)
	{
		strcat(str, " ");
		sprintf(buf, "%d", array[i]);
		strcat(str, buf);
		strcat(str, " ,");
	}
	sprintf(buf, "%d", array[i]);
	strcat(str, buf);
	
	return str;
}




