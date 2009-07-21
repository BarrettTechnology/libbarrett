#include <stdio.h> /* For printf() */

/* Allow us to catch the Ctrl-C signal and exit gracefully */
#include <signal.h>

/* Provides mlockall(), prevent process memory from being swapped out to disk */
#include <sys/mman.h>

#include <libbarrett/wam.h>
#include <libbarrett/os.h>

int done;
void quit()
{
   done = 1;
   return;
}

int main(int argc, char ** argv)
{
   struct bt_wam * wam;
   char buf[256];

   /* Check program arguments */
   if (argc != 2)
   {
      printf("Usage: %s <wam>\n",argv[0]);
      return -1;
   }

   printf("Starting WAM Example 1: Joint Position\n");

   /* Lock all memory, to prevent it from being swapped to disk ... */
   mlockall(MCL_CURRENT | MCL_FUTURE);

   /* Open the WAM given as the first program argument */
   wam = bt_wam_create(argv[1]);
   if (!wam)
   {
      printf("Could not open WAM.\n");
      return -1;
   }

   /* Register the ctrl-c interrupt handler */
   printf("Press Control+C to quit.\n");
   done = 0;
   signal(SIGINT, quit);
   while (!done)
   {
      /* Display the WAM's joint angles on-screen (see **NOTE below) */
      printf("\rPosition (rad) = %s\t", bt_wam_str_jposition(wam,buf));
      fflush(stdout);
      bt_os_usleep(100000); /* Wait a moment */
   }

   printf("\n");
   bt_wam_destroy(wam);
   
   return 0;
}
