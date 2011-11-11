#include <stdio.h> /* For printf() */
#include <string.h>

/* Allow us to catch the Ctrl-C signal and exit gracefully */
#include <signal.h>

/* Provides mlockall(), prevent process memory from being swapped out to disk */
#include <sys/mman.h>

#include <syslog.h>

#include <libbarrett/wam.h>
#include <libbarrett/os.h>

char * vector_format(char * buf, double * vec, int n);

int main(int argc, char ** argv)
{
   struct bt_wam * wam;
   double jmove[] = {0.10, -1.57, 0.79, 1.57, 1.57, -0.79, 0.79};
   double cqmove[] = {-0.25, 0.00, 0.25, 1.00, 0.00, 0.00, 0.00};
   char buf[256];

   /* Check program arguments */
   if (argc != 2)
   {
      printf("Usage: %s <wam>\n",argv[0]);
      return -1;
   }

   /* Lock all memory, to prevent it from being swapped to disk ... */
   mlockall(MCL_CURRENT | MCL_FUTURE);

   /* Turn off output buffering for printf() */
   setvbuf(stdout,0,_IONBF,0);

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

   /* Make sure the WAM is on and in the idle state. */
   printf("\n");
   printf("Starting WAM Example 5: Simple Move.\n");
   printf("\n");
   printf("To continue, ensure the following steps have been taken:\n");
   printf("  a) All WAM power and signal cables are securely fastened.\n");
   printf("  b) The WAM is powered on (receiving power).\n");
   printf("  c) All E-STOPs are released.\n");
   printf("  d) The WAM is in Shift+Idle mode.\n");
   printf("  e) The WAM is in its home (folded) position.\n");
   printf("Press [Enter] to continue.\n");
   while (getchar()!='\n') bt_os_usleep(10000);

   /* Open the WAM given as the first program argument */
   bt_wam_create(&wam,argv[1]);
   if (!wam)
   {
      printf("Could not open WAM.\n");
      return -1;
   }

   /* Let the user activate the WAM
    * before attempting to send torque commands */
   printf("Next, Shift+Activate the system.\n");
   printf("Press [Enter] to turn on gravity compensation.\n");
   while (getchar()!='\n') bt_os_usleep(10000);
   bt_wam_setgcomp(wam,1);

   
   /* Start the joint space move */
   printf("Press [Enter] to start a joint space move,\n");
   printf("  to position <J1, J2, J3, J4, J5, J6, J7>:\n");
   printf("  < %s >\n", vector_format(buf,jmove,bt_wam_dof(wam)));
   printf("  with velocity = %f, acceleration = %f.", 0.5, 0.5);
   while (getchar()!='\n') bt_os_usleep(10000);
   
   bt_wam_control_use_space(wam,"joint");
   bt_wam_set_velocity(wam, 0.5);
   bt_wam_set_acceleration(wam, 0.5);
   bt_wam_moveto(wam, bt_wam_dof(wam), jmove);

   while (!bt_wam_moveisdone(wam)) bt_os_usleep(10000);
   printf("  ... done.\n");

   /* Disable the position constraint (back to just gravity comp) */   
   printf("Press [Enter] to idle the joint space controller.\n");
   while (getchar()!='\n') bt_os_usleep(10000);
   bt_wam_idle(wam);


    /* Start the Cartesian-quaternion space move */
   printf("Press [Enter] to start a Cartesian-xyz-q space move,\n");
   printf("  to position <X, Y, Z, Q1, Q2, Q3, Q4>:\n");
   printf("  < %s >\n", vector_format(buf,cqmove,7));
   printf("  with velocity = %f, acceleration = %f.", 0.2, 0.2);
   while (getchar()!='\n') bt_os_usleep(10000);
   
   bt_wam_control_use_space(wam,"Cartesian-xyz-q");
   bt_os_usleep(1000000);
   bt_wam_set_velocity(wam, 0.2);
   bt_wam_set_acceleration(wam, 0.2);
   bt_wam_moveto(wam, 7, cqmove);

   while (!bt_wam_moveisdone(wam)) bt_os_usleep(10000);
   printf("  ... done.\n");

   /* Disable the position constraint (back to just gravity comp) */   
   printf("Press [Enter] to idle the Cartesian-xyz-q space controller.\n");
   while (getchar()!='\n') bt_os_usleep(10000);
   bt_wam_idle(wam);


   printf("Press [Enter] to move the WAM back home ...");
   while (getchar()!='\n') bt_os_usleep(10000);
   bt_wam_set_velocity(wam, 0.5);
   bt_wam_set_acceleration(wam, 0.5);
   bt_wam_movehome(wam);
   while (!bt_wam_moveisdone(wam)) bt_os_usleep(10000);
   printf("  ... done.\n");
   
   /* Allow the user to re-home and shift-idle the WAM*/
   printf("Idle the WAM with Shift+Idle, and\n");
   printf("  press [Enter] to end the program.\n");
   while (getchar()!='\n') bt_os_usleep(10000);
   
   bt_wam_destroy(wam);
   closelog();
   
   return 0;
}

char * vector_format(char * buf, double * vec, int n)
{
   int i;
   buf[0] = '\0';

   if (n)
      sprintf(buf,"%.4f",vec[0]);

   for (i=1; i<n; i++)
      sprintf(buf + strlen(buf),", %.4f",vec[i]);

   return buf;
}
