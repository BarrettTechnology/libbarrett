/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... posix.hpp
 *  Author ............. vw
 *  Creation Date ...... 3 August 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  C++ wrapper for pthread.h
 *
 * ======================================================================== */
#ifndef POSIX_HPP
#define POSIX_HPP

#include "WamException.hpp"

class Posix
{
private:
   void * (* func)(void *);
   void * arg;

protected: 
   pthread_t tid;
   
public:
   /** Constructor
    * \param func Function thread will execute
    * \param arg Arguments passed to function */
   Posix( void * (* func)(void *), void * arg);
   
   /** Destructor */
   virtual ~Posix();

   void start() throw (WamException);
   
   void stop() throw (WamException);    
};




#endif //POSIX_HPP

