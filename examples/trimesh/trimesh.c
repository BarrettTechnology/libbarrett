/* ======================================================================== *
 *  Module ............. trimesh
 *  File ............... trimesh.c
 *  Author ............. Traveler Hauptman
 *                       Brian Zenowich
 *                       Christopher Dellin
 *  Creation Date ...... 18 Feb 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  NOTES:
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
#include <libbarrett/wam.h>
#include <libbarrett/wam_local.h>
#include <libbarrett/gsl.h>

/* Our own, custom refgen! */
#include "refgen_trimesh.h"

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

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
   struct bt_wam_local * wam_local;
   
   /* My refgen_cylinder */
   struct refgen_trimesh * tri = 0;
   
   /* What to display? */
   enum {
      SCREEN_MAIN,
      SCREEN_HELP
   } screen;

   /* Check arguments */
   if (argc != 2)
   {
      printf("Usage: %s <wam>\n",argv[0]);
      return 1;
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
   
   /* Open the WAM (or WAMs!) */
   bt_wam_create(&wam, argv[1]);
   if (!wam)
   {
      endwin();
      printf("Could not open the WAM.\n");
      exit(-1);
   }
   wam_local = bt_wam_get_local(wam);
   
   /* Make the triangle */
   tri = refgen_trimesh_create("bigcyl.wrl",wam_local->cposition);
   if (!tri)
   {
      bt_wam_destroy(wam);
      endwin();
      printf("Could not create the trimesh.\n");
      exit(-1);
   }
   
   /* Manually set the tool kinematics info
    * (eventually this should come from a config file) */
   /*gsl_matrix_set(wam_local->kin->tool->trans_to_prev, 2,3, 0.183);*/ /* chuck w/ little haptic ball */
   gsl_matrix_set(wam_local->kin->tool->trans_to_prev, 2,3, 0.090); /* opd haptic ball */
   
   /* Start the demo program ... */
   screen = SCREEN_MAIN;
   
   /* Register the ctrl-c interrupt handler
    * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
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

            /* Show controller name (joint space, cartesian space) */
            mvprintw(line++, 0, " Controller: %s",  bt_wam_get_current_controller_name(wam,buf) );

            /* Show GRAVTIY COMPENSATION */
            mvprintw(line++, 0, "GravityComp: %s", bt_wam_isgcomp(wam) ? "On" : "Off" );
            
            /* Show HOLDING */
            mvprintw(line++, 0, "    Holding: %s", bt_wam_is_holding(wam) ? "On" : "Off" );
            
            mvprintw(line++, 0, "     Refgen: %s", bt_wam_refgen_active_name(wam,buf) );
            
            mvprintw(line++, 0, " MoveIsDone: %s", bt_wam_moveisdone(wam) ? "Done" : "Moving" );
            
            mvprintw(line++, 0, "   Teaching: %s", bt_wam_is_teaching(wam) ? "On" : "Off" );
            line++;
            
            /* Show trimesh refgen stuff */
            if (tri)
            {
               mvprintw(line++, 0, "     pos: %s", bt_gsl_vector_sprintf(buf,tri->pos) );
               mvprintw(line++, 0, "      hs: %s", bt_gsl_vector_sprintf(buf,tri->hs) );
               line++;
            }
            
            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", bt_wam_str_jposition(wam,buf) );
            mvprintw(line++, 0, "J Velocity : %s", bt_wam_str_jvelocity(wam,buf) );
            mvprintw(line++, 0, "J Torque   : %s", bt_wam_str_jtorque(wam,buf) );
            line++;

            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, "C Position : %s", bt_wam_str_cposition(wam,buf) );
            
            mvprintw(line,   0, "C Rotation :");
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r1(wam,buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r2(wam,buf) );
            mvprintw(line++, 13, "%s", bt_wam_str_crotation_r3(wam,buf) );
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
      switch (btkey_get())
      {
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
            bt_wam_run(wam);
            break;
         /* tri refgen stuff */
         case 'u': /* Use our surface refgen! */
            if (!tri) break;
            bt_wam_local_refgen_use(wam_local,(struct bt_refgen *)tri);
            break;
         /* end tri refgen stuff */
         default:
            break;
         }
      
      /* Slow this loop down to about 10Hz */
      bt_os_usleep(100000); /* Wait a moment*/
   }
   
   /* Close the WAM */
   bt_wam_destroy(wam);
   
   /* Close ncurses */
   endwin();
   
   /* Close syslog */
   closelog();
   
   return 0;
}
