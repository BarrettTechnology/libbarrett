#ifndef CLIENT_HANDLER_H
#define CLIENT_HANDLER_H



class client_handler
{
    public:
   client_handler(Sockets *);
   virtual ~client_handler();
   static char *  rand_num_gen(int size, char * s);


    private:
   /*fields */
   Sockets * sock;
   

   


};
#endif  //CLIENT_HANDLER_H
