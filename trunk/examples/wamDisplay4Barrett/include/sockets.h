#ifndef SOCKETS_H
#define SOCKETS_H

//C Headers
extern "C" {
#include <netinet/in.h>
#include <sys/time.h>
}
//Defines
#define MAX_QUEUE 11
#define MAXCONNECTIONS 1

enum socket_type {SocketMaster,SocketClient};

/*! \author Pedro Queiros <pquerios@gmail.com>
    \brief Class that represents an endpoint for socket communication.

   *This class haves two distinct Functions:
   * - Implement an internet socket for "Server" purposes - Creates a listening socket and wait a
   * client call request to establish a connection.
   * - Implement an internet socket for "Client" purposes - Creates a socket and does a connection request to a Server.
   *
   * If the connection request is successefuly established, the communication processe is similar to pipes communication.
   *
   *Specifications:
   * - Communication domain: AF_INET;
   * - Sockets-Type: SOCK_STREAM;
   *(more information see man 2 socket)

   TODO improve documentation
*/
class Sockets {

   public:

/*! \brief Server Constructor
*
*Creates a new socket and configures it to be on the listening mode. When a communication request is recieved,
* it is accepted and the listening socket is closed.
*
*   \param local_port Port Number to the listening socket.
*/
//    Sockets(int local_port);




   /*! \brief Server Constructor
    *
    *Creates a new socket and configures it to be on the listening mode. It blocks until a communication request is recieved, or a timeout happens.
    * When the connection is accepted and the listening socket is closed.
    * Do not forget to check the error variable for errors.
    *
    *   \param local_port Port Number to the listening socket.
    *   \param timeout_usec amount of time in micro seconds to wait  if its value is -1 waits forever.
    */
   Sockets(int local_port, long timeout_usec = -1, bool print_errors=false);





/*! \brief Client Constructor
*
* Creates a new socket and does a connection request to the target_port in the target_ip machine
* Do not forget to check the error variable for errors.
*
*   \param target_ip A string pointer to the target ip machine that you want to establish the connection.
*   \param target_port Port Number in the target machine.
*/
   Sockets(char *target_ip,int target_port, bool print_errors=false);




/*! \brief Class Destructor
*
*Closes de Sockets descriptor
*/
   ~Sockets(void);


   void socket_close(void);


     /** \brief Tries to reestablish the connection
    *
    * For Master: stays listenig the listen port until arrives a connections request or timeout happens
    * For Slave:  tries to connect to master imediatly. Time out doesn't apply to it.
    * Do not forget to check the error variable for errors.
    *
    * \param timeout_usec amount of time in micro seconds to wait  if its value is -1 waits forever
     */
   void reconnect(long timeout_usec = -1);



     /** \brief Variable to check for an error during object creation
    *
    * \retval 0 success
    * \retval -1 if an error occured
    * \retval -2 wait for connections time out (only for master)
     */
   int error;

// Data Transimission Functions


/*! \brief Sends Data over the socket
*
*Function used to transmit a message to the other socket
*
*   \param Data A void pointer to the data to be send.
*   \param data_size The Data length
*
*   \retval -1 on error.
*   \retval number_of_characters_sent on success.
*/
   int send (void *Data, const int data_size, const int flags = 0);



/*! \brief  Recieves Data from the socket
*
*Function used to recieve a message from the other socket
*
*   \param Data A pointer to the location where te Data will be stored on success.
*   \param data_size The Data variable length.
*   \param flags Contains the default value "0" (bloking recv calls). Use "MSG_DONTWAIT" to non bloking recv calls.
*
*   \retval -1 on error.
*   \retval 0 when  the  peer  has  performed an orderly shutdown. (only on bloking recv calls)
*   \retval number_of_bytes_recieved on success.
*/
   int recv (void *Data, const int data_size, const int flags = 0);


     /** \brief  Recieves Data from the socket with time out
    *
    *   \param Data A pointer to the location where te Data will be stored on success.
    *   \param data_size The Data variable length.
    *   \param timeout_usec amount of time in micro seconds to wait before returns.
    *
    *   \retval -1 on error.
    *   \retval 0 when  the  peer  has  performed an orderly shutdown.
    *   \retval number_of_bytes_recieved on success.
    *   \retval -2 wait for connections time out reached
     */
   int recvtimeout(void *Data, const int data_size, long timeout_usec);






   private: //internal use functions and variables
   int m_sock;
   sockaddr_in m_addr;
   socket_type type;
   char targetIp[15]; //used only by the client
   int port;
   bool print_errors;

   fd_set selpool;
   timeval timeout;
// Server initialization
  void client_create();
  void server_create(long timeout_usec);
  bool create();
  bool bind ( const int port );
  bool listen ();
  bool accept (int* new_df);

  // Client initialization
  bool connect ( const char* host, const int port );

  bool is_valid() const { return m_sock != -1; }
};
#endif
