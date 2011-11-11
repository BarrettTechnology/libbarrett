/* ======================================================================== *
 *  Module ............. WAM-ROS
 *  File ............... posix.cpp
 *  Author ............. vw
 *  Creation Date ...... 3 August 2009
 *                                                                          *
 *  **********************************************************************  *
 *                                                                          *
 *  C++ wrapper for pthread.h
 *
 * ======================================================================== */


#include <iostream>
#include <pthread.h>

#include "Posix.hpp"


using namespace std;

Posix::Posix( void * (* func)(void *), void * arg)//: func(func), arg(arg)
{
   this->func = func;
   this->arg = arg;
}

Posix::~Posix()
{}

void Posix::start() throw (WamException)
{
   if (pthread_create(&tid, NULL, func, (void *) arg))   
   {
      throw WamException(THREAD_ERROR, "failed to create thread");
   }
   return;
}

void Posix::stop() throw (WamException)
{
   if (pthread_join(tid, NULL) )
   {
      throw WamException(THREAD_ERROR, "failed to join thread");    
   }
   return;
}


