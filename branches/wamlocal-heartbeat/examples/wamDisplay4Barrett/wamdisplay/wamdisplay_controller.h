#ifndef WAMDISPLAY_CONTROLLER_H
#define WAMDISPLAY_CONTROLLER_H

//#include <libbarrett/wam_local.h>
#include <pthread.h>


class wamdisplay_controller
{
   public:
      wamdisplay_controller(double * shared_angle, 
										int shared_finish, 
										pthread_mutex_t * shared_mutex);
      virtual ~wamdisplay_controller();
      void * run(void);
      
	private:
		double * shared_angle;
		int shared_finish;
		pthread_mutex_t * shared_mutex;
	
      bool get_angles(char * angle_str, int num_angles, double * d_array);
      void toggle_state(char c);
      

};






#endif




