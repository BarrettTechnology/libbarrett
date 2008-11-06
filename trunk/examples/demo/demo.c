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
#include <signal.h>
#include <sys/mman.h>

/* Package Dependencies */
#include <curses.h>
#include <syslog.h>
#include <libconfig.h>

/* Include the high-level WAM header file */
#include <libbt/wam.h>
#include <libbt/gsl.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>

/* ------------------------------------------------------------------------ *
 * Key Grabbing Utility Function, for use with ncurses's getch()            */
enum btkey {
   BTKEY_UNKNOWN = -2,
   BTKEY_NOKEY = -1,
   BTKEY_ENTER = 10,
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
   
   /* Get number and letter keys */
   if (c1 >= '0' && c1 <= '9') return c1;
   if (c1 >= 'A' && c1 <= 'Z') return c1;
   if (c1 >= 'a' && c1 <= 'z') return c1;
   
   /* Get special keys */
   switch (c1)
   {
      case 10: return BTKEY_ENTER;
      case 127: return BTKEY_BACKSPACE;
      /* Get extended keyboard chars (eg arrow keys) */
      case 27:
         c2 = getch();
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
         return BTKEY_UNKNOWN;
   }
}


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
   int err;
   
   /* Stuff for starting up the WAM */
   struct config_t cfg;
   struct bt_wam * wam;
   
   /* What to display? */
   enum {
      SCREEN_MAIN,
      SCREEN_HELP
   } screen;
   
   /* Lock memory */
   mlockall(MCL_CURRENT | MCL_FUTURE);
   
   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   
   /* Initialize ncurses */
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();
   
   /* Look for (-q) or (-ns) flags?? */
   
   /* Open the config file */
   config_init(&cfg);
   err = config_read_file(&cfg,"../wam.config");
   if (err != CONFIG_TRUE)
   {
      syslog(LOG_ERR,"libconfig error: %s, line %d\n",
             config_error_text(&cfg), config_error_line(&cfg));
      exit(-1);
   }
   
   /* Open the WAM (or WAMs!) */
   wam = bt_wam_create(config_lookup(&cfg,"wam"));
   if (!wam)
   {
      printf("Could not open the WAM.\n");
      exit(-1);
   }
   
   /* Start the demo program ... */
   screen = SCREEN_MAIN;
   
   /* Register the ctrl-c interrupt handler
    * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
   /*printf("\nPress Ctrl-C to exit...\n");*/
   going = 1;
   
   while(going)
   {
      /* Clear the screen buffer */
      clear();
      
      /* Display the display or help screen */
      switch (screen)
      {
         int line;
         char buf[256];
         case SCREEN_MAIN:
            line = 0;
            
            /* Show HEADER */
            mvprintw(line++, 0, "Barrett Technology - Demo Application\t\tPress 'h' for help");
            line++;
            
            /* Show state controller MODE (joint space, cartesian space) */
            mvprintw(line++, 0, " Controller: %s", wam->con_active->name );
            
            /* Show CONSTRAINT */
#if 0
            mvprintw(line++, 0, "      State: %s", bt_control_mode_name(wam->con_active->get_mode(wam->con_active)) );
            line++;
#endif
            
            /* Show GRAVTIY COMPENSATION */
            mvprintw(line++, 0, "GravityComp: %s", bt_wam_isgcomp(wam) ? "On" : "Off" );
            line++;
            
            /* Show HOLDING */
            mvprintw(line++, 0, "    Holding: %s", wam->con_active->is_holding(wam->con_active) ? "On" : "Off" );
            line++;
            
            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", bt_gsl_vector_sprintf(buf,wam->jposition) );
            mvprintw(line++, 0, "J Torque   : %s", bt_gsl_vector_sprintf(buf,wam->jtorque) );
            line++;
            
            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, "C Position : %s", bt_gsl_vector_sprintf(buf,wam->cposition) );
            mvprintw(line,   0, "C Rotation :");
            {
               int i;
               gsl_vector_view view;
               for (i=0; i<3; i++)
               {
                  view = gsl_matrix_row(wam->crotation,i);
                  mvprintw(line++, 13, "%s", bt_gsl_vector_sprintf(buf,&view.vector));
               }
            }
            line++;
            
            /* Show CURRENT TEACH POINT */
            
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
         case 'x':
         case 'X':
            going = 0;
            break;
         case 'g':
            bt_wam_setgcomp(wam, bt_wam_isgcomp(wam) ? 0 : 1 );
            break;
         case 'h':
            if ( wam->con_active->is_holding(wam->con_active) )
               wam->con_active->idle(wam->con_active);
            else
               wam->con_active->hold(wam->con_active);
            break;
         default:
            break;
      }
      
      /* Slow this loop down to about 10Hz */
      bt_os_usleep(100000); /* Wait a moment*/
   }
   
   /* Close the WAM */
   bt_wam_destroy(wam);
   
   /* Deallocate the configuration */
   config_destroy(&cfg);
   
   /* Close ncurses */
   endwin();
   
   /* Close syslog */
   closelog();
   
   return 0;
}
