#include <stdio.h> /* For printf() */

/* Allow us to catch the Ctrl-C signal and exit gracefully */
#include <signal.h>

/* Provides mlockall(), prevent process memory from being swapped out to disk */
#include <sys/mman.h>

#include <syslog.h>

/* The ncurses library allows us to write text anywhere on the screen */
#include <curses.h>

#include <libbarrett/wam.h>
#include <libbarrett/os.h>

int done;
void quit(int arg)
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

   

   /* Lock all memory, to prevent it from being swapped to disk ... */
   mlockall(MCL_CURRENT | MCL_FUTURE);

   /* Initialize the ncurses screen library */
   initscr(); cbreak(); noecho(); timeout(0); clear();

   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);

   mvprintw(0,0,"Starting WAM Example 2: Gravity Compensation\n");
   
   mvprintw(2,0,"To begin the example, follow the following steps:");
   mvprintw(3,0,"  a) Ensure that all WAM power and signal cables are securely fastened.");
   mvprintw(4,0,"  b) Ensure that the WAM is powered on (receiving power).");
   mvprintw(5,0,"  c) Ensure that all E-Stops are released.");
   mvprintw(6,0,"  d) Place the WAM in Shift+Idle mode.");
   mvprintw(7,0,"  e) Ensure that the WAM is in its home (folded) position.");
   mvprintw(8,0,"Press [Enter] to continue.");
   while (getch()!=10) bt_os_usleep(10000);

   /* Open the WAM given as the first program argument */
   wam = bt_wam_create(argv[1]);
   if (!wam)
   {
      printf("Could not open WAM.\n");
      return -1;
   }

   mvprintw(10,0,"Next, Shift+Activate the system.");
   mvprintw(11,0,"Press [Enter] to turn on gravity compensation.");
   while (getch()!=10) bt_os_usleep(10000);
   bt_wam_setgcomp(wam,1);

   /* Register the ctrl-c interrupt handler */
   /* Clear the screen (ncurses) */
   clear(); refresh();
   done = 0;
   signal(SIGINT, quit);
   while (!done)
   {
      mvprintw(0,0,"Welcome to WAM Example 2: Gravity Compensation\n");
      mvprintw(1,0,"Press Shift+Idle, and Control+C to quit.\n");

      mvprintw(3, 0, "  Joint Position (rad): %s", bt_wam_str_jposition(wam,buf));
      mvprintw(4, 0, "     Joint Torque (Nm): %s", bt_wam_str_jtorque(wam,buf));
      mvprintw(5, 0, "Cartesian Position (m): %s", bt_wam_str_cposition(wam,buf));
      
      refresh(); /* Draw the screen */
      bt_os_usleep(100000); /* Wait a moment */
   }

   bt_wam_destroy(wam);
   closelog();
   endwin();
   
   return 0;
}
