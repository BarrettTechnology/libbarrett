#ifndef SERVER_MANAGER_H
#define SERVER_MANAGER_H

#include <queue>
#include <string>

#include <pthread.h>
//#include <libbarrett/gsl.h>
//#include <libbarrett/wam.h>



enum STATE
{
   IDLE = 0,
     MOVE = 1,
     FORCE = 2,
     TRAJ = 3
};


class server_manager{

   public:   
      /*functions*/
      server_manager(bt_wam * wam, 
                     std::queue<std::string> * pcmd_q, 
                     pthread_mutex_t * pmutex_q);
      virtual ~server_manager();         
      void * run(void *);


      
   private:

      /* wam pointer */
      bt_wam * wam;
      
      /*command queue pointer */
      std::queue<std::string> * pcmd_q;             //command queue

      /* mutex pointer for command queue */
      pthread_mutex_t * pmutex_q;   //= PTHREAD_MUTEX_INITIALIZER; ????
      
      STATE state;
      
      
      /*functions */
      int execute(std::string str);
      bool get_angles(char * angle_str, int num_angles, double * d_array);
      gsl_vector * angles_to_vector(double * darray);
};







#endif
