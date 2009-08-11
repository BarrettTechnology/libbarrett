

#include <iostream>

/* System Libraries */
#include <signal.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>

#include <syslog.h>
#include <curses.h>

#include "../BtWam.hpp"
#include "../Posix.hpp"
//#include <exception>

using namespace std;

void * display(void *);


/* We have a global flag and signal handler
 * to allow the user to close the program
 * with [Control+C] */
int going;
void sigint_handler(int param)
{
   going = 0;
}


int main(int argc, char** argv)
{

#if 1
   char c;

   do
   {
      cout << "Enter 1 if WAM is in folded position" << endl;
      cin >> c;
   } while(c!='1');
   
   do
   {
      cout << "Enter 1 if WAM is in shift-idle" << endl;
      cin >> c;
   } while(c!='1');
   
   do
   {
      cout << "Enter 1 to start program" << endl;
      cin >> c;
   } while(c!='1');
   
   char addr[50] = "wam7";
   Wam wam(addr);



   try 
   {
      wam.init();
   } catch(WamException& e)
   {
      cout << "Exception: " << e.what() << endl;
      exit(-1);
   }
#if 1
   Posix thd( display,(void *) &wam);
   try
   {       
      thd.start();
   } catch(WamException& e)
   {
      cout << "Exception: " << e.what() << endl;
      exit(-1);
   }
#endif
#if 0
   cout << wam.getJointPosition() << endl;
   double * d;
   d = wam.getJointPositionDbl();
   for (int i= 0; i < 7; i++)
      cout << d[i] << endl;
#endif
   while(c != 'x')
   {
      cin >> c;
      
      try 
      {
          switch(c)
          {
             case 'x':
             case 'X':
                going = 0;
                break;
             case 'g':
                wam.setGravityCompensation( (wam.isGravityCompensation())?0:1);
                break;
             case 'h':
                if (wam.isHolding() )
                   wam.idle();
                else
                   wam.hold();
                break;   
             case 'm':
                wam.moveHome();
                break;
             case 'Y':
             try{
                wam.teachStart();
                break;
             }catch (WamException& e)
             {
                cout << "Exception: " << e.what() << endl;
                exit(-1);
             }
             case 'y':
                wam.teachEnd();
                break;
             case '.':
                wam.playback();
                break;
             default:
                break;
          }
      } catch (WamException& e)
      {
         cout << "Exception: " << e.what() << endl;
         cout << "Exiting program" << endl;
         exit(-1);
      }
   }

   //thd.stop();
   
   cout << "bye bye world" << endl;

#endif
   
   return 0;
}


void * display(void * _obj)
{
   Wam * wam = (Wam *) _obj;
   
   /* Initialize ncurses */
   initscr();
   cbreak();
   noecho();
   timeout(0);
   clear();
   
   enum {
      SCREEN_MAIN,
      SCREEN_HELP
   } screen;
   
     screen = SCREEN_MAIN;
     
   /* Register the ctrl-c interrupt handler
    * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
   going = 1;
   
   
   while (going)
   {

      /* Clear the screen buffer */
      clear();
      
      /* Display the display or help screen */
      switch (screen)
      {
         int line;
         case SCREEN_MAIN:
            line = 0;
            
            /* Show HEADER */
            mvprintw(line++, 0, "Barrett Technology - Demo Application\t\tPress 'h' for help");
            line++;

            /* Show controller name (joint space, cartesian space) */
            mvprintw(line++, 0, " Controller: %s",  wam->getCurrentControllerName() );

            /* Show GRAVTIY COMPENSATION */
            mvprintw(line++, 0, "GravityComp: %s", wam->isGravityCompensation() ? "On" : "Off" );
            
            /* Show HOLDING */
            mvprintw(line++, 0, "    Holding: %s", wam->isHolding() ? "On" : "Off" );
            
            mvprintw(line++, 0, "     Refgen: %s", wam->refgenActiveName() );
            
            mvprintw(line++, 0, " MoveIsDone: %s", wam->moveIsDone() ? "Done" : "Moving" );
            
            mvprintw(line++, 0, "   Teaching: %s", wam->isTeaching() ? "On" : "Off" );
            line++;
            
#if 0
            /* Show trimesh refgen stuff */
            if (tri)
            {
               mvprintw(line++, 0, "     pos: %s", bt_gsl_vector_sprintf(buf,tri->pos) );
               mvprintw(line++, 0, "      hs: %s", bt_gsl_vector_sprintf(buf,tri->hs) );
               line++;
            }
#endif        
            /* Show HAPTICS */
            
            /* Show TRAJECTORY */
            
            /* Show NAME */
            
            /* Show JOINT POSTITION + TORQUE */
            mvprintw(line++, 0, "J Position : %s", wam->getJointPosition() );
            mvprintw(line++, 0, "J Velocity : %s", wam->getJointVelocity() );
            mvprintw(line++, 0, "J Torque   : %s", wam->getJointTorque() );
            line++;

            /* Show CARTESION POSITION, ROTATION */
            mvprintw(line++, 0, "C Position : %s", wam->getCartesianPosition() );
            
            mvprintw(line,   0, "C Rotation :");
            mvprintw(line++, 13, "%s", wam->getCartesianRotationRow1() );
            mvprintw(line++, 13, "%s", wam->getCartesianRotationRow2() );
            mvprintw(line++, 13, "%s", wam->getCartesianRotationRow3() );
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
      
      usleep(100000); //10hz
   
   }
         
   /* Close ncurses */
   endwin();
   
   return 0;
}
