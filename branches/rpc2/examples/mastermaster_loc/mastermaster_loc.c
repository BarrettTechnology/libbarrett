/* ======================================================================== *
 *  Module ............. mastermaster
 *  File ............... mastermaster.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 13 Apr 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  NOTES:
 *
 *  REVISION HISTORY:
 *                                                                          *
 * ======================================================================== */

/* System Libraries */
#include <signal.h>
#include <sys/mman.h>

/* Package Dependencies */
#include <curses.h>
#include <syslog.h>
#include <libconfig.h>

/* Include the high-level WAM header file */
#include <libbarrett/wam.h>
#include <libbarrett/wam_custom.h>
#include <libbarrett/wam_local.h>
#include <libbarrett/gsl.h>

/* Our own, custom refgen! */
#include "refgen_mastermaster_loc.h"

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

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
   /* Stuff for starting up the two WAMs */
   int i;
   struct bt_wam * wam[2];
   struct bt_wam_local * wam_local[2];
   
   /* Two refgen_mastermasters */
   struct refgen_mastermaster_loc * mas[2] = {0,0};
   
   /* What to display? */
   enum {
      SCREEN_MAIN,
      SCREEN_HELP
   } screen;
   
   /* Check arguments */
   if (argc < 3)
   {
      printf("Usage: %s <wam1> <wam2>\n",argv[0]);
      return 0;
   }
   
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
   
   /* Open the WAM (or WAMs!) */\
   wam[0] = bt_wam_create(argv[1]);
   wam[1] = bt_wam_create(argv[2]);
   if (!wam[0] || !wam[1])
   {
      endwin();
      printf("Could not open the WAM.\n");
      exit(-1);
   }
   wam_local[0] = bt_wam_get_local(wam[0]);
   wam_local[1] = bt_wam_get_local(wam[1]);
   
   /* Toggle once to get joint controller */
   
   /* Manually set the tool kinematics info
    * (eventually this should come from a config file) */
   /*gsl_matrix_set(wam_local->kin->tool->trans_to_prev, 2,3, 0.183);*/ /* chuck w/ little haptic ball */
   /*gsl_matrix_set(wam_local->kin->tool->trans_to_prev, 2,3, 0.0); /* old haptic ball */
   
   /* Start the demo program ... */
   screen = SCREEN_MAIN;
   
   /* Register the ctrl-c interrupt handler
    * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
   going = 1;
   
   while(going)
   {
      enum btkey key;
      
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

            /* Show controller name (joint space, cartesian space) */
            mvprintw(line,    0, " Controller: %s",  bt_wam_get_current_controller_name(wam[0],buf) );
            mvprintw(line++, 40, " Controller: %s",  bt_wam_get_current_controller_name(wam[1],buf) );

            /* Show GRAVTIY COMPENSATION */
            mvprintw(line,    0, "GravityComp: %s", bt_wam_isgcomp(wam[0]) ? "On" : "Off" );
            mvprintw(line++, 40, "GravityComp: %s", bt_wam_isgcomp(wam[1]) ? "On" : "Off" );
            
            /* Show HOLDING */
            mvprintw(line,    0, "    Holding: %s", bt_wam_is_holding(wam[0]) ? "On" : "Off" );
            mvprintw(line++, 40, "    Holding: %s", bt_wam_is_holding(wam[1]) ? "On" : "Off" );
            
            mvprintw(line,    0, "     Refgen: %s", bt_wam_get_current_refgen_name(wam[0],buf) );
            mvprintw(line++, 40, "     Refgen: %s", bt_wam_get_current_refgen_name(wam[1],buf) );
            
            mvprintw(line,    0, " MoveIsDone: %s", bt_wam_moveisdone(wam[0]) ? "Done" : "Moving" );
            mvprintw(line++, 40, " MoveIsDone: %s", bt_wam_moveisdone(wam[1]) ? "Done" : "Moving" );
            
            mvprintw(line,    0, "   Teaching: %s", bt_wam_is_teaching(wam[0]) ? "On" : "Off" );
            mvprintw(line++, 40, "   Teaching: %s", bt_wam_is_teaching(wam[0]) ? "On" : "Off" );
            line++;

            /* Show mastermaster refgen stuff */
            if (mas[0])
            {
               mvprintw(line,    0, "     power: %.1f", mas[0]->power );
               mvprintw(line+1,  0, "   started: %s", mas[0]->started ? "Yes" : "No" );
            }
            if (mas[1])
            {
               mvprintw(line,   40, "     power: %.1f", mas[1]->power );
               mvprintw(line+1, 40, "   started: %s", mas[1]->started ? "Yes" : "No" );
            }
            if (mas[0] || mas[1])
            {
               line += 3;
            }
            
            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            mvprintw(line++, 0, "Wam 0:");
            line++;
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", bt_wam_str_jposition(wam[0],buf) );
            mvprintw(line++, 0, "J Velocity : %s", bt_wam_str_jvelocity(wam[0],buf) );
            mvprintw(line++, 0, "J Torque   : %s", bt_wam_str_jtorque(wam[0],buf) );
            line++;

            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, "C Position : %s", bt_wam_str_cposition(wam[0],buf) );
            
            mvprintw(line,   0, "C Rotation :");
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r1(wam[0],buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r2(wam[0],buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r3(wam[0],buf) );
            line++;
            
            mvprintw(line++, 0, "Wam 1:");
            line++;
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", bt_wam_str_jposition(wam[1],buf) );
            mvprintw(line++, 0, "J Velocity : %s", bt_wam_str_jvelocity(wam[1],buf) );
            mvprintw(line++, 0, "J Torque   : %s", bt_wam_str_jtorque(wam[1],buf) );
            line++;

            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, "C Position : %s", bt_wam_str_cposition(wam[1],buf) );
            
            mvprintw(line,   0, "C Rotation :");
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r1(wam[1],buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r2(wam[1],buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r3(wam[1],buf) );
            line++;

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
      switch (key)
      {
         int flag;
         case 'x':
         case 'X':
            going = 0;
            break;
         case BTKEY_TAB:
            bt_wam_controller_toggle(wam[0]);
            bt_wam_controller_toggle(wam[1]);
            break;
         case 'g':
            flag = bt_wam_isgcomp(wam[0]);
            bt_wam_setgcomp(wam[0], flag ? 0 : 1 );
            bt_wam_setgcomp(wam[1], flag ? 0 : 1 );
            break;
         case 'h':
            if ( bt_wam_is_holding(wam[0]) )
            {
               bt_wam_idle(wam[0]);
               bt_wam_idle(wam[1]);
            }
            else
            {
               bt_wam_hold(wam[0]);
               bt_wam_hold(wam[1]);
            }
            break;
         case 'm':
            bt_wam_movehome(wam[0]);
            bt_wam_movehome(wam[1]);
            break;
         case 'Y':
            bt_wam_teach_start(wam[0]);
            bt_wam_teach_start(wam[1]);
            break;
         case 'y':
            bt_wam_teach_end(wam[0]);
            bt_wam_teach_end(wam[1]);
            break;
         case '.':
            bt_wam_playback(wam[0]);
            bt_wam_playback(wam[1]);
            break;
         /* mastermaster refgen stuff */
         case 'U': /* Use our mastermaster refgen! */
            if (mas[0] || mas[1]) break;
            mas[0] = refgen_mastermaster_loc_create(wam_local[0]->jposition);
            mas[1] = refgen_mastermaster_loc_create(wam_local[1]->jposition);
            if (!mas[0] || !mas[1])
            {
               syslog(LOG_ERR,"Could not create the mastermaster.");
               break;
            }
            refgen_mastermaster_loc_set_other(mas[0],mas[1]);
            refgen_mastermaster_loc_set_other(mas[1],mas[0]);
            bt_wam_local_refgen_use(wam_local[0],&(mas[0]->base));
            bt_wam_local_refgen_use(wam_local[1],&(mas[1]->base));
            break;
         case 'u':
            if (!mas[0] || !mas[1]) break;
            bt_wam_local_idle(wam_local[0]);
            bt_wam_local_idle(wam_local[1]);
            bt_refgen_destroy((struct bt_refgen *)mas[0]);
            bt_refgen_destroy((struct bt_refgen *)mas[1]);
            mas[0] = 0;
            mas[1] = 0;
            break;
         /* end mastermaster refgen stuff */
         case '0':
         case '1':
         case '2':
         case '3':
         case '4':
         case '5':
         case '6':
         case '7':
         case '8':
         case '9':
            if (!mas[0] || !mas[1]) break;
            refgen_mastermaster_loc_set_power(mas[0],       0.1 * (key - '0'));
            refgen_mastermaster_loc_set_power(mas[1], 1.0 - 0.1 * (key - '0'));
            break;
         case 'a':
            if (!mas[0] || !mas[1]) break;
            refgen_mastermaster_loc_set_power(mas[0], 1.0);
            refgen_mastermaster_loc_set_power(mas[1], 0.0);
            break;
         /* end mastermaster_loc refgen stuff */
         default:
            break;
         }
      
      /* Slow this loop down to about 10Hz */
      bt_os_usleep(100000); /* Wait a moment*/
   }
   
   /* Close the WAM */
   bt_wam_destroy(wam[0]);
   bt_wam_destroy(wam[1]);
   
   /* Close ncurses */
   endwin();
   
   /* Close syslog */
   closelog();
   
   return 0;
}
