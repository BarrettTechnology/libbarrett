#ifndef COMMUNICATION_THREAD_H
#define COMMUNICATION_THREAD_H

class CommunicationThread
{
	public:
		CommunicationThread(double * shared_angle, 
							int shared_finish, 
							pthread_mutex_t * shared_mutex);
		virtual ~CommunicationThread();
		void * run(void);
		
	private:
		double * shared_angle;
		int shared_finish;
		pthread_mutex_t * shared_mutex;
	
		
};

#endif //COMMUNICATION_THREAD_H
