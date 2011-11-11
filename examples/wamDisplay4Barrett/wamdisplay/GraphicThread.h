#ifndef GRAPHIC_THREAD_H
#define GRAPHIC_THREAD_H

//struct arguments;


class GraphicThread
{
	public:
		GraphicThread(double * shared_angle, 
							int shared_finish, 
							pthread_mutex_t * shared_mutex,
							void * struct_ptr);
		virtual ~GraphicThread();
		void * run(void);
		int shared_finish;
		
		
		void read_shared_memory();

	private:
		double * shared_angle;

		pthread_mutex_t * shared_mutex;
		arguments * argument;
//		(struct *)  argument;
			int tempovoo;


	
		

};

#endif //GRAPHIC_THREAD_H


