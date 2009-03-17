/* ======================================================================== *
 *  Module ............. Example 2 - Gravity Compensation
 *  File ............... main.c
 *  Creation Date ...... 26 May 2005
 *  Author ............. Traveler Hauptman
 *  Addtl Authors ...... Brian Zenowich
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  Copyright (C) 2005-2008 Barrett Technology, Inc. <support@barrett.com>
 *                          625 Mount Auburn St
 *                          Cambridge, MA 02138, USA
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY BARRETT TECHNOLOGY, INC AND CONTRIBUTORS
 *  ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL BARRETT
 *  TECHNOLOGY, INC OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 *  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  The views and conclusions contained in the software and documentation
 *  are those of the authors and should not be interpreted as representing
 *  official policies, either expressed or implied, of Barrett Technology.
 *                                                                          *
 * ======================================================================== */

/** \file main.c
    A minimalist program for the WAM that calculates and sends gravity 
    compensation torques.
 
 */

/*==============================*
 * INCLUDES - System Files      *
 *==============================*/
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>
#include <signal.h>
#include <sys/mman.h>
/* The ncurses library allows us to write text anywhere on the screen */
#include <curses.h>

/*==============================*
 * INCLUDES - Project Files     *
 *==============================*/
/*#include "btwam.h"*/
#include <libbt/wam_legacy.h>

/*==============================*
 * GLOBAL file-scope variables  *
 *==============================*/
btrt_thread_struct   rt_thd, wam_thd;
wam_struct           *wam;
int                  startDone;

/*==============================*
 * PRIVATE Function Prototypes  *
 *==============================*/
void Cleanup();
void sigint_handler();
void rt_thread(void *thd);

/*==============================*
 * Functions                    *
 *==============================*/
 
/* If Ctrl-C is pressed, exit gracefully */
void sigint_handler()
{
   Cleanup();
   exit(1);
}

/* The CANbus card must be initialized and called from a realtime thread.
 * The rt_thread is spun off from main() to handle the initial communications.
 */
void rt_thread(void *thd){
   int err;

   /* Probe and initialize the robot actuators */
   err = InitializeSystem();
   if(err) {
      syslog(LOG_ERR, "InitializeSystem returned err = %d", err);
      exit(1);
   }
    
   /* Initialize and get a handle to the robot on the first bus */
   /*if((wam = OpenWAM("../../wam.conf", 0)) == NULL){*/
   if((wam = OpenWAM("wam4", 0)) == NULL){
      syslog(LOG_ERR, "OpenWAM failed");
      exit(1);
   }
   
   /* setSafetyLimits(bus, joint rad/s, tip m/s, elbow m/s);
    * For now, the joint and tip velocities are ignored and
    * the elbow velocity provided is used for all three limits.
    */
   setSafetyLimits(0, 1.5, 1.5, 1.5);  // Limit to 1.5 m/s

   /* Set the puck torque safety limits (TL1 = Warning, TL2 = Critical).
    * Note: The pucks are limited internally to 3441 (see 'MT' in btsystem.c) 
    * Note: btsystem.c bounds the outbound torque to 8191, so entering a
    * value of 9000 for TL2 would tell the safety system to never register a 
    * critical fault.
    */
   setProperty(0, SAFETY_MODULE, TL2, FALSE, 4700);
   setProperty(0, SAFETY_MODULE, TL1, FALSE, 1800);
   
   /* Notify main() thread that the initialization is complete */
   startDone = TRUE;
   
   /* Spin until we are told to exit */
   while (!btrt_thread_done((btrt_thread_struct*)thd)){
      usleep(10000);
   }
   
   /* Remove this thread from the realtime scheduler */
   btrt_thread_exit((btrt_thread_struct*)thd);
}

/* Exit the realtime threads and close the system */
void Cleanup(){
   wam_thd.done = TRUE;
   usleep(10000);
   CloseSystem();
   rt_thd.done = TRUE;
   printf("\n\n");
}

/* Program entry point */
int main(int argc, char **argv)
{
   int   err;        // Generic error variable for function calls
   int   busCount;   // Number of WAMs defined in the configuration file
   char  buf[1024];  // String used by sprint_vn() to convert the joint angle data to text
   int   line;       // Line marker for where to print text on the screen
   
   /* Allow hard real time process scheduling for non-root users */
#ifdef RTAI   
   rt_allow_nonroot_hrt();
#else
   mlockall(MCL_CURRENT | MCL_FUTURE);
   /* Xenomai non-root scheduling is coming soon! */
#endif

   /* Initialize the ncurses screen library */
   initscr(); cbreak(); noecho(); timeout(0); clear();
   atexit((void*)endwin);
   
   /* Initialize syslog */
   openlog("WAM", LOG_CONS | LOG_NDELAY, LOG_USER);
   atexit((void*)closelog);

   /* Register the ctrl-c interrupt handler */
   signal(SIGINT, sigint_handler);

   /* Read the WAM configuration file */
   err = ReadSystemFromConfig("../../wam.conf", &busCount);
   if(err) {
      syslog(LOG_ERR, "ReadSystemFromConfig returned err = %d", err);
      exit(1);
   }
   
   /* Lead the user through a proper WAM startup */
   mvprintw(0,0,"Make sure the all WAM power and signal cables are securely");
   mvprintw(1,0,"fastened, then turn on the main power to WAM and press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   mvprintw(3,0,"Make sure all E-STOPs are released, then press Shift-Idle");
   mvprintw(4,0,"on the control pendant. Then press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   mvprintw(6,0,"Place WAM in its home (folded) position, then press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   
   /* Spin off the RT task to set up the CAN Bus */
   startDone = FALSE;
   btrt_thread_create(&rt_thd, "rtt", 45, (void*)rt_thread, NULL);
   while(!startDone)
      usleep(10000);

   /* Spin off the WAM control loop */
   wam_thd.period = 0.002; // Control loop period in seconds
   btrt_thread_create(&wam_thd, "ctrl", 90, (void*)WAMControlThread, (void*)wam);

   mvprintw(8,0,"Please activate the WAM (press Shift+Activate on the pendant), ");
   mvprintw(9,0,"then press <Enter>");
   while(getch()==ERR)
      usleep(5000);
   
   /* Clear the screen (ncurses) */
   clear(); refresh();
   
   /* Set gravity scale to 1.0g */
   SetGravityComp(wam, 1.0); 

   /* Loop until Ctrl-C is pressed */
   mvprintw(0,0,"Gravity compensation and WAM data display demo");
   while(1) {
      /* Display some interesting WAM data on-screen */
      line = 2;
      
      mvprintw(line, 0, "Robot name = %s     Degrees of Freedom = %d", wam->name, wam->dof); line += 2;
      mvprintw(line, 0, "Joint Position (rad): %s", sprint_vn(buf, wam->Jpos)); ++line;
      mvprintw(line, 0, "Joint Torque (Nm)   : %s", sprint_vn(buf, wam->Jtrq)); ++line;
      mvprintw(line, 0, "Cartesian XYZ (m)   : %s", sprint_vn(buf, (vect_n*)wam->Cpos)); ++line;
#if 0
      mvprintw(line, 0, "Jacobian Matrix\n%s", sprint_mn(buf, wam->robot.J)); line += 7;
      mvprintw(line, 0, "Mass Matrix\n%s", sprint_mn(buf, wam->robot.M)); line += wam->dof + 1;
#endif
      
      ++line;
      mvprintw(line, 0, "To exit, press Shift-Idle on pendant, then hit Ctrl-C");
      
      refresh(); // Draw the screen
      usleep(1E5); // Sleep for 1E5 microseconds or 0.1 seconds
   }
   
   return(0); 
}
