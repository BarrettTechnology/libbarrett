#include <queue>
#include <string>
#include <iostream>

extern "C" {
/* System Libraries */
#include <signal.h>
#include <sys/mman.h>
#include <iostream>

/* Package Dependencies */
#include <syslog.h>
#include <libconfig.h>
#include <curses.h>

#include <pthread.h>


/* Include the high-level WAM header file */
//#include <libbarrett/wam.h>
//#include <libbarrett/os.h>
//#include <libbarrett/wam_custom.h>
#include <libbarrett/wam_local.h>
//#include <libbarrett/gsl.h>

/* Our own, custom refgen! */
#include "refgen_trimesh.h"
}


/* Custom Dependencies */
#include "../sockets/sockets.h"
#include "server_manager.h"
#include "server_controller.h"
#include "server_handler.h"

#define SOCKET_CONNECT_TIMEOUT_USEC -1
#define TEXTGUI 0
#define SERVER_CONTROLLER_ON 1
/* ------------------------------------------------------------------------ */

static const int socket_port=2021;

/**main server thread. should be launched on WAM machine locally.
initializes:
    - a static server manager
    - 1 server controller per client
    - 1 server handler per client */

using namespace std;

void * init_server_manager(void *);
void * initServerController(void * );

void textUI(struct bt_wam *, struct bt_wam_local *);

void * initServerHandler(void * );

 /* Key Grabbing Utility Function, for use with ncurses's getch()            */
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
char btkey_get()
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


//TODO MOVE
//server_manager serverManager;

/* We have a global flag and signal handler
* to allow the user to close the program
* with [Control+C] */
int going;                                 //include in header file??
int connected;

void sigint_handler(int param)
{
   going = 0;
}

/* What to display? */
enum MODE {
   SCREEN_MAIN,
   SCREEN_HELP
};
MODE screen;

/* My refgen_cylinder */
struct refgen_trimesh * tri = 0;

#if SERVER_CONTROLLER_ON
/*create global command queue */
queue<string> cmd_q;
#endif

/* Program entry point */
int main(int argc, char ** argv)
{
   /* Stuff for starting up the WAM */
   struct bt_wam * wam;
   struct bt_wam_local * wam_local;



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
   char str[50] = "wam7";
   wam = bt_wam_create(str);
   if (!wam)
   {
      printw("Could not open the Wam.");
      endwin();
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


   /*spin off thread to process command queue */

   //   pthread_t serverManager_t;
   //   pthread_create( &serverManager_t, NULL, init_server_manager, (void *) NULL);


   /* Start the demo program ... */
   screen = SCREEN_MAIN;

   /* Register the ctrl-c interrupt handler
   * to close the WAM nicely */
   signal(SIGINT, sigint_handler);
   /* Loop until Ctrl-C is pressed */
   going = 1;
   printw("Initializing server...\n");
   refresh();
   while(going)
   {
      /* Initialize server socket */
      connected = 0;
      
      /* will lock until client connection made */
      Sockets serverSocket(socket_port); //, SOCKET_CONNECT_TIMEOUT_USEC);

      //TODO handle destroying of threads, have print statement

      if(!serverSocket.error)      //if no error upon connection, proceed
         connected = 1;

      /* make threads */
      pthread_t serverHandlerThd; //, serverControllerThd;


      /* spin off server handler thread */
      server_handler * serverHandler = new server_handler(&serverSocket, wam, &connected, &going);
      pthread_create(&serverHandlerThd, NULL, initServerHandler, serverHandler);   

      
#if SERVER_CONTROLLER_ON
      /* spin off server controller thread */
      pthread_t serverControllerThd; 
      server_controller * serverController = new server_controller(&serverSocket, &cmd_q, &connected, &going);
      pthread_create(&serverControllerThd, NULL, initServerController, serverController);
#endif

      /*server text interface */
      textUI(wam, wam_local);
      
      clear();          //MUST BE INCLUDED else server disconnects ::weird::
      pthread_join(serverHandlerThd, NULL);
      printw("disconnected from client\n");
      printw("returning to listening...\n");  
      refresh();
      
 //     /* wait for thread to terminate */
   //   pthread_join(thread1, NULL);      

        
   }          
           
   
   /* Close the WAM */
   bt_wam_destroy(wam);

   /* Close ncurses */
   endwin();

   /* Close syslog */
   closelog();

   return 0;
}

#if SERVER_CONTROLLER_ON
void * init_server_manager(void * params)
{
   //struct thread_data * my_data = (thread_data *) params;
   //server_manager serverManager(my_data->wam, my_data->pcmd_q, my_data->pmutex_q);
   
   
   
   
   return 0;
}

void * initServerController(void * _serverControllerObj)
{
   //struct thread_data * my_data = (thread_data *)params;
   //server_controller serverController(my_data->sock, my_data->pcmd_q, my_data->pconnected, my_data->pgoing);
   server_controller * serverControllerObj = (server_controller *) _serverControllerObj;
   cout << "starting server controller" << endl;
   void * threadResult = serverControllerObj->run();
   delete serverControllerObj;
   return threadResult;
} 

#endif

/*needs wam pointer and socket pointer */
void * initServerHandler(void * _serverHandlerObj)
{
   //struct thread_data * my_data = (thread_data *)params;
   //serverHandlerObj = new serverHandler(my_data->sock, my_data->wam, my_data->pconnected, my_data->pgoing);
   server_handler * serverHandlerObj = (server_handler *) _serverHandlerObj;
   cout << "starting server handler " << endl;
   void * threadResult = serverHandlerObj->run();
   delete serverHandlerObj;

   //lock with mutex???
   connected = 0;
   
   return threadResult;
}

void textUI(struct bt_wam * wam, struct bt_wam_local * wam_local)
{

   while(connected && going)
   {

#if TEXTGUI
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
#endif
         /* Display the screen */
         refresh();
         

      
         /* Grab a key */
         char key = btkey_get();
         switch (key)
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

}




