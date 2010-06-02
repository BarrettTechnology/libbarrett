#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include <sys/types.h> /* For select() */
#include <unistd.h> /* For exec() */

#include "common.h"

int main(int argc, char* argv[])
{
   int i;
   fd_set set;
   struct timeval timeout;
   char input[100];
   int chars_read;
   char * tail;
   char listing = 0;
   
   if (argc == 2  &&  strcmp(argv[1], "-l") == 0) {
      listing = 1;
   }
   
   /* Create the missing sentinel in function call
UDP socket */
   if (init_socket())
   {
      printf("Couldn't create UDP socket for broadcast transmission.\n");
      close_socket();
      return -1;
   }
   
   /* Send the broadcast message */
   if (send_broadcast() == -1)
   {
      printf("Error sending packet ...\n");
      close_socket();
      return -1;
   }
   
   if (!listing) {
      printf("Awaiting responses ...\n");
   }
   
   /* Loop until we get no responses in 1 second ... */
   while (1)
   {
      FD_ZERO( &set);
      FD_SET( get_socket(), &set );
      timeout.tv_sec = 1;
      timeout.tv_usec = 0;
      
      select(FD_SETSIZE, &set, 0, 0, &timeout);
      
      if ( FD_ISSET( get_socket(), &set ) )
      {
/*         int num;*/
         
         i = receive_packet();
         if (i < 0)
         {
            printf("Error receiving packet.\n");
            close_socket();
            return -1;
         }

         printf("Response #%d: %s\n", i+1, list_get_display_str(i));
         
         continue;
      }
      else
      {
         break;
      }
   }

   
   if (listing) {
      close_socket();
      return 0;
   }
   
   if (!list_get_size())
   {
      printf("No WAMs found.\n");
      close_socket();
      return 0;
   }
   
   
   i = -1; /* Which index? */
   while (i == -1)
   {
      int ii;
      
      printf("Press [Enter] to exit, or enter the response number of the WAM you want to ssh to");
      printf(" (e.g. \"1\" or \"3\")\n");
      printf("> ");
      
      fgets(input,100,stdin);
      chars_read = strlen(input);
      /* Chomp the newline */
      if (input[chars_read-1] == '\n')
      {
         input[chars_read-1] = '\0';
         chars_read--;
      }
      
      if (!chars_read) /* nothing entered */
         break;
      
      ii = (int) strtol(input,&tail,10);
      if (tail == input)
         continue;
      if (ii < 1 || ii > list_get_size())
         continue;
      i = ii - 1;
      break;
   }
   
   close_socket();

   if (i == -1)
   {
      return 0;
   }

   
   /* Get the username */
   printf("username: ");
   fgets(input,100,stdin);
   chars_read = strlen(input);
   /* Chomp the newline */
   if (input[chars_read-1] == '\n')
   {
      input[chars_read-1] = '\0';
      chars_read--;
   }
   
   execlp("ssh","ssh","-l",input,list_get_ipstr(i),(char *)0);
   return 0;
}
