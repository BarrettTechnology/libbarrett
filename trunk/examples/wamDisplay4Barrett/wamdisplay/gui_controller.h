#ifndef GUI_HANDLER_H
#define GUI_HANDLER_H

#include <libbarrett/wam.h>
//#include <libbarrett/wam_local.h>
#include <pthread.h>

class gui_controller
{
   public:
      gui_controller(double[], pthread_mutex_t *);
      virtual ~gui_controller();
      void * run(void *);
      
   private:   
      bool get_angles(char * angle_str, int num_angles, double * d_array);
      double * darray;
      pthread_mutex_t * pmutex;
   
};






#endif




