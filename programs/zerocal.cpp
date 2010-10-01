/*
 * zerocal.cpp
 *
 *  Created on: Apr 8, 2010
 *      Author: dc
 */



#include <string.h>

#include <syslog.h>
#include <curses.h>
#include <gsl/gsl_math.h> /* For M_PI */

#include <barrett/cdlbt/wambot_phys.h>
#include <barrett/cdlbt/bus.h>
#include <barrett/cdlbt/gsl.h>
#include <barrett/cdlbt/os.h>

#include <barrett/systems.h>
#include <barrett/wam.h>


using namespace barrett;

const int DOF = 7;
const double T_s = 0.002;
BARRETT_UNITS_TYPEDEFS(DOF);


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
   if (32 <= c1 && c1 <= 126) return (enum btkey)c1;

   /* Get special keys */
   switch (c1)
   {
      case BTKEY_TAB:
      case BTKEY_ENTER:
      case BTKEY_BACKSPACE:
            return (enum btkey)c1;
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
         return BTKEY_UNKNOWN;
   }
}




/* ------------------------------------------------------------------------ *
 * Mode Zero                                                                */

/* We need to be able to grab the magnetic encoder values
 * from a realtime thread */
struct bt_bus *bus;
struct bt_os_thread * magenc_thd;
int * mz_mechisset;
int * mz_counts;
int * mz_magvals;
double * mz_angles;
int mz_magvals_get;

/* A new rt thread which just gets the encoder positions */
static void magenc_thd_function(struct bt_os_thread * thread)
{
   int i;

   /* Detect puck versions */
   for (i=0; i<DOF; i++)
   {
      long vers;
      long role;
      long cts;
      bt_bus_get_property(bus, i+1, bus->p->VERS, &vers);
      bt_bus_get_property(bus, i+1, bus->p->ROLE, &role);
      bt_bus_get_property(bus, i+1, bus->p->CTS, &cts);
      mz_mechisset[i] = ((vers >= 118) && (role & 256)) ? 1 : 0;
      mz_counts[i] = (int)cts;
   }

   while (!bt_os_thread_isdone(thread))
   {

      long reply;
      if (mz_magvals_get)
      {
         for (i=0; i<DOF; i++) if (mz_mechisset[i])
         {
            reply = 1000;
            bt_bus_get_property(bus, i+1,bus->p->MECH, &reply);
            mz_magvals[i] = (int)reply;
            mz_angles[i] = 2.0 * M_PI * reply / mz_counts[i];
         }
         mz_magvals_get = 0;
      }
      bt_os_usleep(100000);
   }

   bt_os_thread_exit(thread);

   return;
}

int main()
{
   int i;           /* For iterating through the pucks */
   int n;
   int done;

   /* GUI stuff */
   enum MODE {
      MODE_TOZERO,
      MODE_CANCEL,
      MODE_PRINTVALS,
      MODE_JSELECT,
      MODE_EDIT
   } mode;
   int joint;
   int decplace;
   gsl_vector * jangle;

   char newhome[80];
   char zeromag[80];


   /* Initialize the ncurses screen library */
   initscr(); cbreak(); noecho(); timeout(0); clear();

   openlog("bt-wam-calibrate", LOG_CONS | LOG_NDELAY, LOG_USER);

   mvprintw(0,0,"Starting Zero Calibration Mode");

   /* Ensure the WAM is set up, power cycled, and in the home position */
   mvprintw(2,0,"To begin the calibration, follow the following steps:");
   mvprintw(3,0,"  a) Ensure that all WAM power and signal cables are securely fastened.");
   mvprintw(4,0,"  b) Ensure that the WAM is powered on (receiving power).");
   mvprintw(5,0,"  c) E-STOP the WAM (Important!).");
   mvprintw(6,0,"  d) Release all E-STOPs.");
   mvprintw(7,0,"  e) Place the WAM in Shift+Idle mode.");
   mvprintw(8,0,"  f) Carefully ensure that the WAM is in its home (folded) position.");
   mvprintw(9,0,"Press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) bt_os_usleep(10000);

   /* Open the WAM */
	libconfig::Config config;
	config.readFile("/etc/barrett/wam7.conf");

	// remove existing zerocal information, if present
	libconfig::Setting& llSetting = config.lookup("wam.low_level");
	if (llSetting.exists("zeroangle")) {
		llSetting.remove(llSetting["zeroangle"].getIndex());
		syslog(LOG_ERR, "Ignoring previous zeroangle entry.");
	}


	systems::RealTimeExecutionManager rtem(T_s);
	systems::System::defaultExecutionManager = &rtem;

	Wam<DOF> wam(config.lookup("wam"));
	wam.jpController.setControlSignalLimit(jp_type()); // disable torque saturation because gravity comp isn't on
	bus = wam.wam.wambot->bus;

	rtem.start();


   n = DOF;

   /* Spin off the magenc thread, which also detecs puck versions into mechset */
   /* EEK! Why must this be such high priority? */
   mz_mechisset = (int *)malloc( n * sizeof(int) );
   mz_counts    = (int *)malloc( n * sizeof(int) );
   mz_magvals   = (int *)calloc( n, sizeof(int) );
   mz_angles    = (double *)malloc( n * sizeof(double) );
   mz_magvals_get = 0;
   bt_os_thread_create(&magenc_thd, BT_OS_RT, "mag", 91, magenc_thd_function, 0); /* This is global! */
   if (!magenc_thd)
   {
      free(mz_mechisset);
      free(mz_counts);
      free(mz_magvals);
      free(mz_angles);
      rtem.stop();
      endwin();
      closelog();
      printf("Could not create realtime magenc thread.");
      return -1;
   }

   /* Allow the user to shift-activate */
   mvprintw(11,0,"Place the WAM in Shift+Activate mode,");
   mvprintw(12,0,"and press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) bt_os_usleep(10000);

   /* Hold position */
//   bt_wam_hold(wam);

   jp_type prev(wam.getJointPositions());
   jp_type cur;

   /* Start the user interface */
   mode = MODE_TOZERO;
   joint = 0;
   decplace = 4;
   jangle = gsl_vector_alloc(n);
   wam.getJointPositions().copyTo(jangle);

   clear();
   done = 0;
   while (!done)
   {
      int j;
      int line;
      enum btkey key;

//      mvprintw(0, 0, bt_gsl_vector_sprintf(buf,wam_local->con_active->reference));

      /* Display the Zeroing calibration stuff ... */
      mz_magvals_get = 1;

      line = 2;
      if (mode == MODE_TOZERO)
      {
         attron(A_BOLD);
         mvprintw(line++, 0, "--> ] Move To Current Zero");
         attroff(A_BOLD);
      }
      else
         mvprintw(line++, 0, "    ] Move To Current Zero");
      if (mode == MODE_CANCEL)
      {
         attron(A_BOLD);
         mvprintw(line++, 0, "--> ] Cancel Calibration");
         attroff(A_BOLD);
      }
      else
         mvprintw(line++, 0, "    ] Cancel Calibration");
      if (mode == MODE_PRINTVALS)
      {
         attron(A_BOLD);
         mvprintw(line++, 0, "--> ] Print Calibrated Values and Exit");
         attroff(A_BOLD);
      }
      else
         mvprintw(line++, 0, "    ] Print Calibrated Values and Exit");
      if (mode == MODE_JSELECT)
      {
         attron(A_BOLD);
         mvprintw(line, 0, "--> ] Joint:");
         attroff(A_BOLD);
      }
      else
         mvprintw(line, 0, "    ] Joint:");
      mvprintw(line+1, 5, "-------");
      if (mode == MODE_EDIT) attron(A_BOLD);
      mvprintw(line+2, 5, "   Set:");
      if (mode == MODE_EDIT) attroff(A_BOLD);
      mvprintw(line+3, 5, "Actual:");
      mvprintw(line+5, 5, " Motor:");
      mvprintw(line+6, 5, "MagEnc:");
      for (j=0; j<n; j++)
      {
         if ((mode == MODE_JSELECT || mode == MODE_EDIT) && j==joint)
         {
            attron(A_BOLD);
            mvprintw(line+0, 13 + 9*j, "[Joint %d]",j+1);
            attroff(A_BOLD);
         }
         else
            mvprintw(line+0, 13 + 9*j, " Joint %d ",j+1);
         /* total with 8, 5 decimal points (+0.12345) */
         if ( mode==MODE_EDIT && j==joint )
         {
            int boldplace;
            mvprintw(line+1, 13 + 9*j, " _._____ ",j+1);
            mvprintw(line+2, 13 + 9*j, "% 08.5f ",gsl_vector_get(jangle,j));
            boldplace = decplace + 1;
            if (decplace) boldplace++;
            mvprintw(line+1, 13 + 9*j + boldplace,"x");
            mvchgat(line+2, 13 + 9*j+boldplace, 1, A_BOLD, 0, NULL );
         }
         else
         {
            mvprintw(line+1, 13 + 9*j, " ------- ",j+1);
            mvprintw(line+2, 13 + 9*j, "% 08.5f ",gsl_vector_get(jangle,j));
         }
//         mvprintw(line+3, 13 + 9*j, "% 08.5f ",gsl_vector_get(wam_local->jposition,j));
         mvprintw(line+3, 13 + 9*j, "% 08.5f ",gsl_vector_get(wam.getJointPositions().asGslType(),j));
         mvprintw(line+5, 13 + 9*j, " Motor %d",j+1);
         if (mz_mechisset[j])
         {
            mvprintw(line+6, 13 + 9*j, "   %05.3f",mz_angles[j]);
            mvprintw(line+7, 13 + 9*j, "    %04d",mz_magvals[j]);
         }
         else
            mvprintw(line+6, 13 + 9*j, "   (None)",mz_magvals[j]);

      }
      refresh();

      /* Wait a bit ... */
      bt_os_usleep(5E4);
      key = btkey_get();

      /* If the user is in menu mode, work the menu ... */
      if (mode != MODE_EDIT) switch (key)
      {
         case BTKEY_UP:
        	mode = (enum MODE)((int)mode - 1);
            if ((signed int)mode < 0) mode = (enum MODE)0;
            break;
         case BTKEY_DOWN:
         	mode = (enum MODE)((int)mode + 1);
            if (mode > MODE_JSELECT) mode = MODE_JSELECT;
            break;
         case BTKEY_ENTER:
            switch (mode)
            {
               case MODE_TOZERO:
//                  if (!bt_wam_moveisdone(wam)) break;
                  gsl_vector_set_zero(jangle);
                  cur.copyFrom(jangle);
                  wam.moveTo(prev, jv_type(0.0), cur, false, 0.5, 0.5);
                  prev = cur;
//                  bt_wam_local_moveto_vec(wam_local,jangle);
                  break;
               case MODE_CANCEL:
                  done = -1;
                  break;
               case MODE_PRINTVALS:
//                  if (!bt_wam_moveisdone(wam)) break;
                  done = 1;
                  break;
               case MODE_JSELECT:
                  mode = MODE_EDIT;
                  decplace = 4;
                  break;
               case MODE_EDIT:
                  break;
            }
            break;
         default:
            if (mode == MODE_JSELECT) switch (key)
            {
               case BTKEY_LEFT:
                  joint--;
                  if (joint < 0) joint = 0;
                  break;
               case BTKEY_RIGHT:
                  joint++;
                  if (joint >= n) joint = n - 1;
                  break;
               default:
                  break;
            }
            break;
      }
      /* We're in joint edit mode */
      else switch (key)
      {
         case BTKEY_LEFT:
            decplace--;
            if (decplace < 0) decplace = 0;
            break;
         case BTKEY_RIGHT:
            decplace++;
            if (decplace > 5) decplace = 5;
               break;
         case BTKEY_BACKSPACE:
            mode = MODE_JSELECT;
            break;
         /* Actually do the moves */
         case BTKEY_UP:
//            if (!bt_wam_moveisdone(wam)) break;
            *(gsl_vector_ptr(jangle,joint)) += pow(10,-decplace);
            cur.copyFrom(jangle);
            wam.moveTo(prev, jv_type(0.0), cur, false, 0.5, 0.5);
            prev = cur;
//            err = bt_wam_local_moveto_vec(wam_local,jangle);
//            if (err)
//            {
//               syslog(LOG_ERR,"Error with moveto: %d.",err);
//               *(gsl_vector_ptr(jangle,joint)) -= pow(10,-decplace);
//            }
            break;
         case BTKEY_DOWN:
//            if (!bt_wam_moveisdone(wam)) break;
            *(gsl_vector_ptr(jangle,joint)) -= pow(10,-decplace);
            cur.copyFrom(jangle);
            wam.moveTo(prev, jv_type(0.0), cur, false, 0.5, 0.5);
            prev = cur;
//            err = bt_wam_local_moveto_vec(wam_local,jangle);
//            if (err)
//            {
//               syslog(LOG_ERR,"Error with moveto: %d.",err);
//               *(gsl_vector_ptr(jangle,joint)) += pow(10,-decplace);
//            }
            break;
         default:
            break;
      }
   }

   if (done == 1)
   {
      gsl_vector * vec;
      /* Save the new home location */
      vec = gsl_vector_alloc(n);
      gsl_vector_memcpy(vec,wam.wam.wambot->base.home);
      gsl_vector_sub(vec,jangle);
      sprintf(newhome,"( %05.3f",gsl_vector_get(vec,0));
      for (i=1; i<n; i++)
         sprintf(newhome+strlen(newhome),", %05.3f",gsl_vector_get(vec,i));
      sprintf(newhome+strlen(newhome)," );");
      gsl_vector_free(vec);

      /* Save the zeromag values */
      for (i=0; i<n; i++) if (!mz_mechisset[i]) mz_angles[i] = -1.0;
      sprintf(zeromag,"( %05.3f",mz_angles[0]);
      for (i=1; i<n; i++)
         sprintf(zeromag+strlen(zeromag),", %05.3f",mz_angles[i]);
      sprintf(zeromag+strlen(zeromag)," );");
   }

   /* Re-fold, print, and exit */
   clear();
   mvprintw(0,0,"Moving back to the park location ...");
   refresh();
   wam.moveHome();
//   bt_wam_movehome(wam);
//   while (!bt_wam_moveisdone(wam)) bt_os_usleep(10000);
   mvprintw(1,0,"Shift+Idle, and press [Enter] to continue.");
   refresh();
   while (btkey_get()!=BTKEY_ENTER) bt_os_usleep(10000);

   bt_os_thread_stop(magenc_thd);
   bt_os_thread_destroy(magenc_thd);

   rtem.stop();

   /* Stop ncurses ... */
   endwin();

   if (done == 1)
   {
      /* Print the results */
      printf("\n");
      printf("Zeroing calibration ended.\n");
      printf("\n");
      for (i=0; i<n; i++) if (!mz_mechisset[i])
      {
         printf("Note: Some (or all) of your pucks do not support absolute\n");
         printf("position measurement, either because they do not use magnetic\n");
         printf("encoders, or because they have not been updated to firmware r118.\n");
         printf("\n");
         break;
      }
      printf("Copy the following lines into your wam.config file,\n");
//      printf("near the top, in the %s{} group.\n",wamname);
      printf("Make sure it replaces the old home = ( ... ); definition.\n");
      printf("--------\n");
      printf("      # Calibrated zero values ...\n");
      printf("      home = %s\n",newhome);
      for (i=0; i<n; i++) if (mz_mechisset[i])
      { printf("      zeroangle = %s\n",zeromag); break; }
      printf("--------\n");
//      {
//         FILE * logfile;
//         logfile = fopen("cal-zero.log","w");
//         if (logfile)
//         {
//            fprintf(logfile,"      # Calibrated zero values ...\n");
//            fprintf(logfile,"      home = %s\n",newhome);
//            for (i=0; i<n; i++) if (mz_mechisset[i])
//            { fprintf(logfile,"      zeromag = %s\n",zeromag); break; }
//            fclose(logfile);
//            printf("This text has been saved to cal-zero.log.\n");
//            printf("\n");
//         }
//         else
//         {
//            syslog(LOG_ERR,"Could not write to cal-zero.log.");
//            printf("Error: Could not write to cal-zero.log.\n");
//            printf("\n");
//         }
//      }
      printf("Note that you must E-Stop (or power-cycle) your WAM\n");
      printf("for the calibrated settings to take effect!\n");
      printf("\n");
   }

   free(mz_mechisset);
   free(mz_counts);
   free(mz_magvals);
   free(mz_angles);
   gsl_vector_free(jangle);

   return 0;
}
