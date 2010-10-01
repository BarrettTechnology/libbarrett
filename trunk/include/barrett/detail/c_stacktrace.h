/*
 * c_stack_trace.h
 *
 *  Created on: Oct 26, 2009
 *      Author: dc
 */

#ifndef C_STACKTRACE_H_
#define C_STACKTRACE_H_

#ifdef __cplusplus
extern "C" {
#endif


void c_print_stacktrace();
void c_syslog_stacktrace();


#ifdef __cplusplus
}
#endif

#endif /* C_STACKTRACE_H_ */
