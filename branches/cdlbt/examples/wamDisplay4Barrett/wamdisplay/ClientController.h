#ifndef CLIENT_CONTROLLER_H
#define CLIENT_CONTROLLER_H

//#include <libbarrett/wam_local.h>
#include <pthread.h>

/* This thread object is instantiated when connecting with remote WAM. 
   It retreives joint positions and passes angle values to the GUI */


class ClientController
{
   public:
   //! Constructor
   /**
      \param shared_angle Thread shared doubles array storing joint positions
      \param shared_finish Thread shared integer (not used, included for consistency)
      \param shared_mutex Thread shared mutex used to lock shared_angle       */
      ClientController(double * shared_angle, int shared_finish, pthread_mutex_t * shared_mutex);
      virtual ~ClientController();

   //! Thread mainloop
      void * run(void);
      
   private:
      double * shared_angle;
      int shared_finish;
      pthread_mutex_t * shared_mutex;
   
      bool get_angles(char * angle_str, int num_angles, double * d_array);
      void toggle_state(char c);
      

};






#endif




