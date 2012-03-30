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
#include <stdlib.h> /* For atof() */
#include <string.h>
#include <signal.h>
#include <sys/mman.h>

#include <malloc.h>
#include <unistd.h> /* For usleep() */

/* Package Dependencies */
#include <curses.h>
#include <syslog.h>

/* Include the high-level WAM header file */
#include "wam.h"
/*#include <libbt/os.h>*/ /* for bt_os_usleep() */

/*
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
*/

#define INPUT_MAX 20

/* ------------------------------------------------------------------------ *
 * Key Grabbing Utility Function, for use with ncurses's getch()            */
enum btkey {
   BTKEY_UNKNOWN = -2,
   BTKEY_NOKEY = -1,
   BTKEY_TAB = 9,
   BTKEY_ENTER = 10,
   BTKEY_ESCAPE = 27,
   BTKEY_BACKSPACE = 127,
   BTKEY_UP = 256,
   BTKEY_DOWN = 257,
   BTKEY_LEFT = 258,
   BTKEY_RIGHT = 259
};
enum btkey btkey_get()
{
   int c1,c2,c3;
   
   /* Get the key from ncurses */
   c1 = getch();
   if (c1 == ERR) return BTKEY_NOKEY;
   
   /* Get all keyboard characters */
   if (32 <= c1 && c1 <= 126) return c1;
   
   /* Get special keys */
   switch (c1)
   {
      case BTKEY_TAB:
      case BTKEY_ENTER:
      case BTKEY_BACKSPACE:
            return c1;
      /* Get extended keyboard chars (eg arrow keys) */
      case 27:
         c2 = getch();
         if (c2 == ERR) return BTKEY_ESCAPE;
         if (c2 != 91) return BTKEY_UNKNOWN;
         c3 = getch();
         switch (c3)
         {
            case 65: return BTKEY_UP;
            case 66: return BTKEY_DOWN;
            case 67: return BTKEY_RIGHT;
            case 68: return BTKEY_LEFT;
            default: return BTKEY_UNKNOWN;
         }
      default:
         syslog(LOG_ERR,"Unknown Key: %d\n",c1);
         return BTKEY_UNKNOWN;
   }
}

/* User-input string */
int get_string(int line, char * format, char * buf);
int string_to_vector(char * buf, double * vec, int * len);

/* We have a global flag and signal handler
 * to allow the user to close the program
 * with [Control+C] */
int going;
void sigint_handler()
{
   going = 0;
}

/* Program entry point */
int main(int argc, char ** argv)
{
   /* Stuff for starting up the WAM */
   struct bt_wam * wam;

   /* What to display? */
   enum {
      SCREEN_MAIN,
      SCREEN_HELP
   } screen;
   
   /* Check arguments */
   if (argc < 2)
   {
      printf("Usage: %s <wam>\n",argv[0]);
      return 0;
   }

   /* Lock memory */
   mlockall(MCL_CURRENT | MCL_FUTURE);

   /* Initialize syslog */
   openlog("demo", LOG_CONS | LOG_NDELAY, LOG_USER);

   /* Initialize ncurses */
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();
   /* Look for (-q) or (-ns) flags?? */

   /* Open the WAM (or WAMs!) */
   bt_wam_create(&wam,argv[1]);
   if (!wam)
   {
      endwin();
      closelog();
      printf("Could not open the WAM.\n");
      return -1;
   }

   /* Start the demo program ... */
   screen = SCREEN_MAIN;

   /* Register the ctrl-c interrupt handler
    * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
   going = 1;

   while(going)
   {
      int line;
      line = 0;
      
      /* Clear the screen buffer */
      clear();
      
      /* Display the display or help screen */
      switch (screen)
      {
         char buf[256];
         case SCREEN_MAIN:
            
            /* Show HEADER */
            mvprintw(line++, 0, "Barrett Technology - Demo Application\t\tPress 'h' for help");
            line++;

            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, " J Position: %s", bt_wam_str_jposition(wam,buf) );
            mvprintw(line++, 0, " J Velocity: %s", bt_wam_str_jvelocity(wam,buf) );
            mvprintw(line++, 0, " J Torque  : %s", bt_wam_str_jtorque(wam,buf) );
            line++;
   #if 0
            mvprintw(line++, 0, " J Reference: %s", bt_gsl_vector_sprintf(buf,wam->con_joint->reference) );
            line++;
   #endif
            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, " C Position: %s", bt_wam_str_cposition(wam,buf) );
            
            mvprintw(line,   0, " C Rotation:");
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r1(wam,buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r2(wam,buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r3(wam,buf) );
            line++;

            /* Show controller name (joint space, cartesian space) */
            mvprintw(line++, 0, " Controller: %s", bt_wam_get_current_controller_name(wam,buf) );
            mvprintw(line++, 0, "      Space: %s", bt_wam_get_current_controller_space(wam,buf) );
            mvprintw(line++, 0, "   Position: %s", bt_wam_str_con_position(wam,buf) );
            line++;

            /* Show GRAVTIY COMPENSATION */
            mvprintw(line+0, 0, "   GravityComp: %s", bt_wam_isgcomp(wam) ? "On" : "Off" );
            
            /* Show HOLDING */
            mvprintw(line+1, 0, "       Holding: %s", bt_wam_is_holding(wam) ? "On" : "Off" );
            mvprintw(line+2, 0, "      Teaching: %s", bt_wam_is_teaching(wam) ? "On" : "Off" );

            mvprintw(line+0, 25, " Loaded Refgen: %s", bt_wam_refgen_loaded_name(wam,buf) );
            mvprintw(line+1, 25, " Active Refgen: %s", bt_wam_refgen_active_name(wam,buf) );
            
            mvprintw(line+2, 25, "    MoveIsDone: %s", bt_wam_moveisdone(wam) ? "Done" : "Moving" );
            
            line+=4;

            break;
         case SCREEN_HELP:
            line = 0;
            mvprintw(line++, 0, "Help Screen - (press 'h' to toggle)");
            line++;
            mvprintw(line++, 0, "g - toggle gravity compensation");
            break;
      }
      
      /* Display the screen */
      refresh();
      
      /* Grab a key */
      switch (btkey_get())
      {
         char inbuf[200];
         double invector[10];
         int inveclen;
         
         case 'x':
         case 'X':
            going = 0;
            break;
         case BTKEY_TAB:
            bt_wam_controller_toggle(wam);
            break;
         case 'g':
            bt_wam_setgcomp(wam, bt_wam_isgcomp(wam) ? 0 : 1 );
            break;
         case 'h':
            if ( bt_wam_is_holding(wam) )
               bt_wam_idle(wam);
            else
               bt_wam_hold(wam);
            break;
         case 'M':
            bt_wam_movehome(wam);
            break;
         case 'm':
            if (get_string(line,"    Move To: < %s >",inbuf))
            {
               string_to_vector(inbuf, invector, &inveclen);
               if (inveclen)
                  bt_wam_moveto(wam,inveclen,invector);
            }
            break;
         case 'Y':
            bt_wam_teach_start(wam);
            break;
         case 'y':
            bt_wam_teach_end(wam);
            break;
         case '.':
            bt_wam_run(wam);
            break;
         case 's':
            if (get_string(line,"Save To File: %s",inbuf))
               bt_wam_refgen_save(wam,inbuf);
            break;
         case 'l':
            if (get_string(line,"Load From File: %s",inbuf))
               bt_wam_refgen_load(wam,inbuf);
            break;
         case 'c':
            bt_wam_refgen_clear(wam);
            break;
         default:
            break;
         }
      
      /* Slow this loop down to about 10Hz */
      usleep(100000); /* ~10Hz */
   }

   /* Close the WAM */
   bt_wam_destroy(wam);

   /* Close ncurses */
   endwin();

   /* Close syslog */
   closelog();
   
   return 0;
}

int get_string(int line, char * format, char * buf)
{
   buf[0] = '\0';
   while (1)
   {
      enum btkey key;
      key = btkey_get();

      /* Print the line as it is now */
      mvprintw(line,0,format,buf);

      if (key == BTKEY_NOKEY)
      {
         usleep(50000); /* ~20hz */
         continue;
      }

      /* Are we done? */
      if (key == BTKEY_ENTER)
         return strlen(buf); /* Success */
      if (key == BTKEY_ESCAPE)
         return 0;

      /* Add a character */
      if (32 <= key && key <= 126)
      {
         buf[strlen(buf)+1] = '\0';
         buf[strlen(buf)] = key;
         continue;
      }

      /* Remove a character */
      if (key == BTKEY_BACKSPACE && strlen(buf))
      {
         buf[strlen(buf)-1] = '\0';
         continue;
      }
   }
   return 0;
}

int string_to_vector(char * buf, double * vec, int * len)
{
   char * value;

   (*len) = 0;

   value = strtok(buf,",");
   while (value)
   {
      vec[(*len)] = atof(value);
      (*len)++;
      value = strtok(0,",");
   }

   return 0;
}
