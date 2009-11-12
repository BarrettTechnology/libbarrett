/*
 * exception.cpp
 *
 *  Created on: Nov 10, 2009
 *      Author: dc
 */


#include <exception>
#include "./stacktrace.h"
#include "../exception.h"


namespace barrett {


void (*oldTerminate)();  // pointer to the system's default terminate function

void myTerminate() {
	print_stacktrace();
	oldTerminate();
}

void installExceptionHandler()
{
	oldTerminate = std::set_terminate(myTerminate);
}


}
