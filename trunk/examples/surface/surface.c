/* ======================================================================== *
 *  Module ............. surface
 *  File ............... surface.c
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
#include <libbt/wam.h>
#include <libbt/gsl.h>

/* We also manufacture our own refgens! */
#include <libbt/refgen.h>

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#define INPUT_MAX 20

#define SURF_A_RADIUS (0.0005)
#define SURF_B_RADIUS (0.0005)

/* Here we define our own refgen */
struct refgen_surface
{
   /* Include the base */
   struct bt_refgen base;
   gsl_vector * cpos; /* current, saved */
   
   gsl_vector * start; /* mine, constant */
   gsl_vector * temp; /* test 3-vector to try */
   
   /* 2-D parameters */
   double a;
   double b;
};

static int destroy(struct bt_refgen * base);
static int get_start(struct bt_refgen * base, gsl_vector ** start);
static int get_total_time(struct bt_refgen * base, double * time);
static int get_num_points(struct bt_refgen * base, int * points);
static int start(struct bt_refgen * base);
static int eval(struct bt_refgen * base, gsl_vector * ref);
static const struct bt_refgen_type refgen_surface_type = {
   "surface",
   &destroy,
   &get_start,
   &get_total_time,
   &get_num_points,
   &start,
   &eval
};
const struct bt_refgen_type * refgen_surface = &refgen_surface_type;

/* Generic parameterization */
void func(double a, double b, gsl_vector * pos)
{
   double r2;
   r2 = a*a + b*b;
   gsl_vector_set(pos,0,a+0.5);
   gsl_vector_set(pos,1,b);
   if (r2 < 0.04)
      gsl_vector_set(pos,2, -4*r2);
   else
      gsl_vector_set(pos,2, -4*0.04);
}

/* Functions */
struct refgen_surface * refgen_surface_create(gsl_vector * cpos)
{
   struct refgen_surface * r;
   r = (struct refgen_surface *) malloc(sizeof(struct refgen_surface));
   if (!r) return 0;
   r->base.type = refgen_surface;
   
   /* save the cartesian position */
   r->cpos = cpos;
   
   r->start = gsl_vector_calloc(3);
   func( 0.0, 0.0, r->start );
   
   r->temp = gsl_vector_alloc(3);
   
   return r;
}
static int destroy(struct bt_refgen * base)
{
   struct refgen_surface * r = (struct refgen_surface *) base;
   gsl_vector_free(r->start);
   gsl_vector_free(r->temp);
   free(r);
   return 0;
}
static int get_start(struct bt_refgen * base, gsl_vector ** start)
{
   struct refgen_surface * r = (struct refgen_surface *) base;
   (*start) = r->start;
   return 0;
}
static int get_total_time(struct bt_refgen * base, double * time)
{
   (*time) = 0.0;
   return 0;
}
static int get_num_points(struct bt_refgen * base, int * points)
{
   (*points) = 1;
   return 0;
}
static int start(struct bt_refgen * base)
{
   struct refgen_surface * r = (struct refgen_surface *) base;
   r->a = 0.0;
   r->b = 0.0;
   return 0;
}
static int eval(struct bt_refgen * base, gsl_vector * ref)
{
   int i;
   double a, b;
   double error;
   double new_a_error, new_b_error;
   double a_radius, b_radius;
   struct refgen_surface * r = (struct refgen_surface *) base;
   /* We have r->cpos, the current position */
   
   /* Our initial best guess is a, b. */
   a = r->a;
   b = r->b;
   
   a_radius = SURF_A_RADIUS;
   b_radius = SURF_B_RADIUS;
   
   /* Do a binary search through parameters (a,b) */
   for (i=0; i<5; i++)
   {
      /* Evaluate current error */
      func(a,b, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      error = gsl_blas_dnrm2( r->temp );
      
      /* Test a */
      func(a+a_radius,b, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_a_error = gsl_blas_dnrm2( r->temp );
      
      /* Test b */
      func(a,b+b_radius, r->temp);
      gsl_blas_daxpy( -1.0, r->cpos, r->temp );
      new_b_error = gsl_blas_dnrm2( r->temp );
      
      a_radius /= 2;
      b_radius /= 2;
      
      /* Adjust parameters */
      if (new_a_error < error) a += a_radius;
      else                     a -= a_radius;
      
      if (new_b_error < error) b += b_radius;
      else                     b -= b_radius;
   }
     
   func(a,b,ref);
   
   r->a = a;
   r->b = b;
   
   return 0;
}

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
   int err;
   
   /* Stuff for starting up the WAM */
   struct config_t cfg;
   struct bt_wam * wam;
   
   struct refgen_surface * surf;
   
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
   
   /* Make my "surface" refgen */
   surf = refgen_surface_create(wam->cposition);
   
   /* Manually set the tool kinematics info
    * (eventually this should come from a config file) */
   gsl_matrix_set(wam->kin->tool->trans_to_prev, 2,3, 0.208); /* chuck w/ black rod */
   
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
            mvprintw(line++, 0, " Controller: %s", wam->con_active->type->name );

            /* Show GRAVTIY COMPENSATION */
            mvprintw(line++, 0, "GravityComp: %s", bt_wam_isgcomp(wam) ? "On" : "Off" );
            
            /* Show HOLDING */
            mvprintw(line++, 0, "    Holding: %s", bt_wam_is_holding(wam) ? "On" : "Off" );
            
            mvprintw(line++, 0, "     Refgen: %s", bt_wam_get_current_refgen_name(wam) );
            
            mvprintw(line++, 0, " MoveIsDone: %s", bt_wam_moveisdone(wam) ? "Done" : "Moving" );
            
            mvprintw(line++, 0, "   Teaching: %s", bt_wam_is_teaching(wam) ? "On" : "Off" );
            line++;

            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", bt_gsl_vector_sprintf(buf,wam->jposition) );
            mvprintw(line++, 0, "J Velocity : %s", bt_gsl_vector_sprintf(buf,wam->jvelocity) );
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
            bt_wam_playback(wam);
            break;
         case 'i':
            /* Inject our surface refgen! */
            bt_wam_refgen_use(wam,(struct bt_refgen *)surf);
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
