/*
 * ca_macro.h
 *
 *  Created on: Sep 21, 2009
 *      Author: dc
 */

#ifndef CA_MACRO_H_
#define CA_MACRO_H_

// copied from: http://greeness2008.blogspot.com/2008/10/google-c-coding-style.html
// originally from Google?
//
// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName)		\
		TypeName(const TypeName&);				\
		void operator=(const TypeName&)


#endif /* CA_MACRO_H_ */
