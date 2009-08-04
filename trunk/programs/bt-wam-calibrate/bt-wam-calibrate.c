/* ======================================================================== *
 *  Module ............. demo
 *  File ............... demo.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 14 Oct 2005
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  NOTES:
 *    This is the primary WAM demo application. It shows off many
 *    features of the WAM library.
 *    This application is also used for diagnostics and testing.
 *
 *  REVISION HISTORY:
 *    2008 Sept 16 - CD
 *      Ported from btsystem to libbt, copied from btdiag to demo
 *                                                                          *
 * ======================================================================== */

/** \file demo.c
    Primary WAM demo application.
 */

/* System Libraries */
#include <string.h>   /* strncasecmp(), etc */
#include <sys/mman.h>

/* Package Dependencies */
#include <curses.h>

#include "os.h"

#include "btkey.h"

/* Functions in other files */
int do_zero(char * wamname);
int do_gravity(char * wamname);

/* ------------------------------------------------------------------------ *
 * Program Entry Point -- Choose Calibration Routine                        */
int main(int argc, char **argv)
{
   int err;
   enum {
      MODE_ZERO = 0, /* Determine the zero/home positions */
      MODE_MU, /* Determine the mu vectors */
      MODE_EXIT
   } mode;
   
   /* This keeps this program's memory from being swapped out, reducing
    * the chance of disruptions from other programs on the system. */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
   /* Parse command-line arguments */
   if (   (argc == 1)
       || (    (argc >= 2)
            && (    !strncmp(argv[1],  "h",1)
                 || !strncmp(argv[1], "-h",2)
                 || !strncmp(argv[1],"--h",3)
               )
          )
      )
   {
      printf("Usage: %s <wam> [options]\n",argv[0]);
      printf("\n");
      printf("Options:\n");
      printf("\n");
      printf("  h, -h, --help, ...    Print this message and exit.\n");
      printf("\n");
      printf("  z, -z, --zero, ...    Run the joint-angle zeroing calibration routine.\n");
      printf("                        Requires puck firmware version r118 and magnetic\n");
      printf("                        encoders for full functionality.\n");
      printf("\n");
      printf("  g, -g, --gravity ...  Run the gravity calibration routine.\n");
      printf("\n");
      return 0;
   }
   
   /* Figure out which calibration routine the user wants to do */
   if ((argc >= 3) && ( !strncmp(argv[2],  "z",1)
                     || !strncmp(argv[2], "-z",2)
                     || !strncmp(argv[2],"--z",3) ))
      mode = MODE_ZERO;
   else
   /* Is it "gravity" or "mu" ? */
   if ((argc >= 3) && ( !strncmp(argv[2],  "g",1)
                     || !strncmp(argv[2], "-g",2)
                     || !strncmp(argv[2],"--g",3) ))
      mode = MODE_MU;
   else
   /* OK, let the user choose */
   {
      int chosen = 0;

      /* Initialize the ncurses screen library */
      initscr(); cbreak(); noecho(); timeout(0); clear();
      
      mvprintw(0,0,"Barrett WAM Calibration Program.");
      mvprintw(2,0,"Choose a calibration option from the menu below:");
      mvprintw(4,4,"Calibrate Zero Position");
      mvprintw(5,4,"Calibrate Gravity Compensation");
      mvprintw(6,4,"Exit");
      mode = MODE_ZERO;
      while (!chosen)
      {
         /* Display the currently selected mode */
         if (mode == MODE_ZERO) mvprintw(4,0,"-->");
         else                   mvprintw(4,0,"   ");
         if (mode == MODE_MU)   mvprintw(5,0,"-->");
         else                   mvprintw(5,0,"   ");
         if (mode == MODE_EXIT) mvprintw(6,0,"-->");
         else                   mvprintw(6,0,"   ");
         /* Wait for a while */
         refresh();
         bt_os_usleep(5000);
         /* Get user input */
         switch (btkey_get())
         {
            case BTKEY_UP:    mode--; if ((int)mode < 0)    mode = 0;         break;
            case BTKEY_DOWN:  mode++; if (mode > MODE_EXIT) mode = MODE_EXIT; break;
            case BTKEY_ENTER: chosen = 1;                                     break;
            default:                                                          break;
         }
      }
      endwin();
   }

   err = -1;
   if (mode == MODE_ZERO) err = do_zero(argv[1]);
   if (mode == MODE_MU)   err = do_gravity(argv[1]);
   
   return err;
}
