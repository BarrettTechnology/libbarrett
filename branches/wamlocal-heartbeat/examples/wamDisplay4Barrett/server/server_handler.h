#ifndef SERVER_HANDLER_H
#define SERVER_HANDLER_H


class server_handler
{
   public:
      server_handler(Sockets * sock, struct bt_wam * wam, int * pconnected, int * pgoing, int state);
      virtual ~server_handler();
      void * run(void);
      
   private:
      Sockets * sock;
      bt_wam * wam;
      int * pconnected;
      int * pgoing;
	  int state;
};











#endif
