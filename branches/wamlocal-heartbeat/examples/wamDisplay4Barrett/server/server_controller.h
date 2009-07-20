#ifndef SERVER_CONTROLLER_H
#define SERVER_CONTROLLER_H


class server_controller
{
   public:
      server_controller(Sockets * sock, pthread_mutex_t * pmutex_q, std::queue<std::string> * pcmd_q, int * pconnected, int * pgoing);
      virtual ~server_controller();
      void * run(void);
      
      


   private:
      Sockets * sock;
      pthread_mutex_t * pmutex_q;
      std::queue<std::string> * pcmd_q;
      int * pconnected;
      int * pgoing;

};




#endif
