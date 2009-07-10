#include <iostream>
extern "C" {
#include <sys/select.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
}
#include "sockets.h"

using namespace std;


Sockets::Sockets(int local_port, long timeout_usec,  bool print_errors)
       :error(0),
    m_sock(-1),
        type(SocketMaster),
   port(local_port),
   print_errors(print_errors)
   
{
   server_create(timeout_usec);
   
}


Sockets::Sockets(char *target_ip, int target_port,  bool print_errors)
       :error(0),
   m_sock(-1),
        type(SocketClient),
        port(target_port),
   print_errors(print_errors)
{
   strcpy(targetIp,target_ip);
   client_create();
}


Sockets::~Sockets()
{
   if (m_sock!=-1)
            close(m_sock);
}

void Sockets::socket_close(void)
{
   close(m_sock);
}

void Sockets::reconnect(long timeout_usec)
{
   error=0;
   close(m_sock);
   if(type==SocketMaster)
   {
      server_create(timeout_usec);
   }
   else
   {
      client_create();
   }
}



void Sockets::client_create()
{
   cout << "creating client" << endl;
   int cre = create();
   cout << "create is " << cre<< endl;
   if(!cre)
   {
      if(print_errors)
         perror(" socket creation");
      error=-1;
   }
   int con = connect (targetIp,port);
   cout << "con is " << con << " ip is " << targetIp << " port is " << port << endl;
   if(!con)
   {
      if(print_errors)
         perror(" socket connecting");
      error=-1;
   }
}

void Sockets::server_create(long timeout_usec)
{
   bool created = create();

   if(!created)
   {
      if(print_errors)
         perror(" socket creation");
      error=-1;
   }
   
   bool bp = bind(port);

   if(!bp)
   {
      if(print_errors)
         perror(" socket binding");
      error=-1;
   }

   bool lis = listen();
   if(!lis)
   {
      if(print_errors)
         perror(" socket listenning");
      error=-1;
   }


   int new_fd=-1;
   if (timeout_usec== -1)
   {
      cout << "listening on port " << port << "..." << endl;
      bool in = accept (&new_fd);
      if(!in)
      {
   
         if(print_errors)
            perror(" socket accepting");
         error=-1;
      }
      close(m_sock);
      m_sock=new_fd;
   }
   else
   {

      timeout.tv_sec=0;
      timeout.tv_usec=timeout_usec;

      FD_ZERO(&selpool);
      FD_SET(m_sock, &selpool);

      int ret=select(m_sock+1, &selpool,NULL,NULL,&timeout);
      if(ret==0)
         error=-2; //time out occured
      else
         if(ret==-1)
            error=-1; //an error occured
      else{ // a connection request was caught
         if(!accept (&new_fd)){
            if(print_errors)
            perror(" socket accepting");
         error=-1;
         }
         close(m_sock);
         m_sock=new_fd;
      }
   }
}





bool Sockets::create()
{
cout << "af and ss are " << AF_INET << endl;
cout << SOCK_STREAM << endl;

  m_sock = socket ( AF_INET, SOCK_STREAM, 0);

cout << "create m_sock is " << m_sock << endl;

  if ( ! is_valid() )
    return false;

 int on = 1;
  if ( setsockopt ( m_sock, SOL_SOCKET, SO_REUSEADDR, ( const char* ) &on, sizeof ( on ) ) == -1 )
    return false;



  return true;
}



bool Sockets::bind ( const int port )
{

  if ( ! is_valid() )
    {
      return false;
    }



  m_addr.sin_family = AF_INET;
  m_addr.sin_addr.s_addr = INADDR_ANY;
  m_addr.sin_port = htons ( port );

  int bind_return = ::bind ( m_sock,
              ( struct sockaddr * ) &m_addr,
              sizeof ( m_addr ) );


  if ( bind_return == -1 )
    {
      return false;
    }

  return true;
}


bool Sockets::listen()
{
  if ( ! is_valid() )
    {
      return false;
    }

  int listen_return = ::listen ( m_sock, MAXCONNECTIONS );


  if ( listen_return == -1 )
    {
      return false;
    }

  return true;
}


bool Sockets::accept (int* new_df)
{
  int addr_length = sizeof ( m_addr );
   *new_df= ::accept ( m_sock, ( sockaddr * ) &m_addr, ( socklen_t * ) &addr_length );

  if ( *new_df <= 0 )
    return false;
  else
    return true;
}


bool Sockets::connect ( const char* host, const int port )
{
   cout << "hello?  " << endl;
   
  if ( ! is_valid() ) 
  {
   cout << "begin false" << endl;   
     return false;
  }
  
  m_addr.sin_family = AF_INET;
  m_addr.sin_port = htons ( port );

  int status = inet_pton ( AF_INET, host, &m_addr.sin_addr );

  if ( errno == EAFNOSUPPORT ) return false;

  status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );
cout << "msock is " << m_sock << endl;
cout << "sockaddr is " << &m_addr << endl;
cout << "sizeof maddy " << sizeof(m_addr) << endl;
cout << "status: " << status << endl;

  if ( status == 0 )
    return true;
  else
  {
	strerror(errno);
//	cout << buf << endl; 
	printf("String Error is: %s\n",strerror(errno));
//	cout << errno << endl;
   cout << "end false" << endl;
    return false;
  }
}


int Sockets::send (void* Data, const int data_size, const int flags)
{   
//   cout << "msock is " << m_sock << endl;
//   cout << "data is " << Data << endl;
//   cout << "data_size " << data_size << endl;
  return ::send ( m_sock, Data, data_size, MSG_NOSIGNAL | flags );
}


int Sockets::recv (void* Data, const int data_size, const int flags)
{
   return ::recv ( m_sock, Data, data_size,flags);
}


int
Sockets::recvtimeout(void *Data, const int data_size, long timeout_usec)
{

   timeout.tv_sec=0;
   timeout.tv_usec=timeout_usec;

   FD_ZERO(&selpool);
   FD_SET(m_sock, &selpool);

   int ret=select(m_sock+1, &selpool,NULL,NULL,&timeout);
   if(ret==0)
      return -2; //time out occured

   if(ret==-1)
      return -1; //an error occured

   // data avalieble before time out
   return recv (Data,data_size);
}
