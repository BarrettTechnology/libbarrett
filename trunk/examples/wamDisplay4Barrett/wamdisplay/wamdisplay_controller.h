#ifndef WAMDISPLAY_CONTROLLER_H
#define WAMDISPLAY_CONTROLLER_H

//#include <libbarrett/wam_local.h>
#include <pthread.h>

enum STATE 
{
   VIEW,
   CONTROL
};

class wamdisplay_controller
{
   public:
      wamdisplay_controller(double[], pthread_mutex_t *);
      virtual ~wamdisplay_controller();
      void * run(void *);
      
   private:   
      double * darray;
      pthread_mutex_t * pmutex;
      //   STATE state;
   
      bool get_angles(char * angle_str, int num_angles, double * d_array);
      void toggle_state(char c);
      

};






#endif




