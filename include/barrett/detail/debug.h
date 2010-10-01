/*
 * debug.h
 *
 *  Created on: Nov 10, 2009
 *      Author: dc
 */

#ifndef DEBUG_H_
#define DEBUG_H_


#include <iostream>

#define DEBUG_MARK  \
		std::cerr << "DEBUG:" __FILE__ ":" << __LINE__ << std::endl


#endif /* DEBUG_H_ */
