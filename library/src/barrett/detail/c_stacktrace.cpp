/*
 * stacktrace.c
 *
 *  Created on: Oct 26, 2009
 *      Author: dc
 */

#include "stacktrace.h"
#include "c_stacktrace.h"

void c_print_stacktrace() {
	print_stacktrace();
}

void c_syslog_stacktrace() {
	syslog_stacktrace();
}
