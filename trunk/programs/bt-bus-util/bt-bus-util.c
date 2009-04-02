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
#include <malloc.h>

/* Package Dependencies */
#include <curses.h>
#include <syslog.h>
#include <libconfig.h>

/* Include the high-level WAM header file */
#include "bus.h"
#include "os.h" /* for bt_os_usleep() */

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


/* A realtime thread to service the bus */
struct bt_os_thread * bus_thread;
struct bt_bus * bus;
int setup_done;
int setup_err;

void bus_thread_func(struct bt_os_thread * thd)
{
   config_t cfg;
   config_setting_t * set;
   
   /* Open bus 0 */
   config_init(&cfg);
   set = config_setting_add(config_root_setting(&cfg),"port",CONFIG_TYPE_INT);
   config_setting_set_int(set,0);
   bus = bt_bus_create( config_root_setting(&cfg), bt_bus_UPDATE_POS_ONLY );
   config_destroy(&cfg);
   if (!bus)
   {
      setup_err = 1;
      setup_done = 1;
      bt_os_thread_exit(thd);
      return;
   }
   
   setup_err = 0;
   setup_done = 1;
   
   while (!bt_os_thread_done(thd))
   {
      bt_os_usleep(100000);
   }
   
   bt_bus_destroy(bus);
   
   bt_os_thread_exit(thd);
   return;
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
   /* What to display? */
   enum {
      SCREEN_MAIN,
      SCREEN_HELP
   } screen;
   
   int i;
   int cur_column;
   int cur_puckid;

   /* Lock memory */
   mlockall(MCL_CURRENT | MCL_FUTURE);

   /* Initialize syslog */
   openlog("demo", LOG_CONS | LOG_NDELAY, LOG_USER);

   /* Do bus setup */
   setup_done = 0;
   bus_thread = bt_os_thread_create(BT_OS_RT, "BUS", 90, &bus_thread_func, 0);
   while (!setup_done) bt_os_usleep(1000);
   if (setup_err)
   {
      printf("Could not open bus.\n");
      return 1;
   }
   
   /* Initialize ncurses */
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();

   /* Start the demo program ... */
   screen = SCREEN_MAIN;

   /* Register the ctrl-c interrupt handler
    * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
   going = 1;
   
   /* Initialize to the first puckid */
   cur_puckid = 0;
   for (i=0; i<bus->pucks_size; i++)
   if (bus->puck[i])
   {
      cur_puckid = i;
      break;
   }
   
   cur_column = 1;

   while(going)
   {
      enum btkey key;
      
      /* Clear the screen buffer */
      clear();
      
      /* Display the display or help screen */
      switch (screen)
      {
         int line;
         /*char buf[256];*/
         case SCREEN_MAIN:
            line = 0;
            
            /* Show HEADER */
            mvprintw(line++, 0, "Barrett Technology - Bus Utility Program\t\tPress 'h' for help");
            line++;
            
            /* Show a list of pucks */
            mvprintw(line++, 0, "List of Pucks:");
            for (i=0; i<bus->pucks_size; i++)
            if (bus->puck[i])
               mvprintw(line++, 3, "%s Puck %d, VERS %d",
                        i == cur_puckid ? "-->" : "   ",
                        i, bus->puck[i]->vers);
            line++;
            
            mvprintw(line++, 0, "List of Watches:");
            
#if 0

            /* Show controller name (joint space, cartesian space) */
            mvprintw(line++, 0, " Controller: %s", bt_wam_get_current_controller_name(wam,buf) );
            
            /* Show GRAVTIY COMPENSATION */
            mvprintw(line++, 0, "GravityComp: %s", bt_wam_isgcomp(wam) ? "On" : "Off" );
            
            /* Show HOLDING */
            mvprintw(line++, 0, "    Holding: %s", bt_wam_is_holding(wam) ? "On" : "Off" );
            
            mvprintw(line++, 0, "     Refgen: %s", bt_wam_get_current_refgen_name(wam,buf) );
            
            mvprintw(line++, 0, " MoveIsDone: %s", bt_wam_moveisdone(wam) ? "Done" : "Moving" );
            
            mvprintw(line++, 0, "   Teaching: %s", bt_wam_is_teaching(wam) ? "On" : "Off" );
            line++;

            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", bt_wam_str_jposition(wam,buf) );
            mvprintw(line++, 0, "J Velocity : %s", bt_wam_str_jvelocity(wam,buf) );
            mvprintw(line++, 0, "J Torque   : %s", bt_wam_str_jtorque(wam,buf) );
            line++;
   #if 0
            mvprintw(line++, 0, "J Reference: %s", bt_gsl_vector_sprintf(buf,wam->con_joint->reference) );
            line++;
   #endif
            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, "C Position : %s", bt_wam_str_cposition(wam,buf) );
            
            mvprintw(line,   0, "C Rotation :");
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r1(wam,buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r2(wam,buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r3(wam,buf) );
            line++;
#endif
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
      key = btkey_get();
      
      switch (cur_column)
      {
         case 1:
            switch (key)
            {
               case BTKEY_UP:
                  for (i=cur_puckid-1; i>0; i--)
                  if (bus->puck[i])
                  {
                     cur_puckid = i;
                     break;
                  }
                  break;
               case BTKEY_DOWN:
                  for (i=cur_puckid+1; i<bus->pucks_size; i++)
                  if (bus->puck[i])
                  {
                     cur_puckid = i;
                     break;
                  }
                  break;
               case '1': case '2': case '3': case '4': case '5':
               case '6': case '7': case '8': case '9':
                  i = key - '0';
                  if (i < bus->pucks_size && bus->puck[i])
                     cur_puckid = i;
                  break;
               default:
                  break;
            }
      }
         
      /* Do generic stuff */
      switch (key)
      {
         case 'x':
         case 'X':
            going = 0;
            break;
#if 0
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
         case 'm':
            bt_wam_movehome(wam);
            break;
         case 'Y':
            bt_wam_teach_start(wam);
            break;
         case 'y':
            bt_wam_teach_end(wam);
            break;
         case '.':
            bt_wam_playback(wam);
            break;
#endif
         default:
            break;
         }
      
      /* Slow this loop down to about 10Hz */
      bt_os_usleep(10000); /* Wait a moment*/
   }
   
   /* Kill the bus thread */
   bt_os_thread_stop(bus_thread);
   bt_os_thread_destroy(bus_thread);
   
   /* Close ncurses */
   endwin();

   /* Close syslog */
   closelog();

   return 0;
}
