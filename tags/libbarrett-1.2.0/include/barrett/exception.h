/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/** Exception handling utilities.
 *
 * @file exception.h
 * @date Nov 10, 2009
 * @author Dan Cody
 */


#ifndef BARRETT_EXCEPTION_H_
#define BARRETT_EXCEPTION_H_


namespace barrett {

/** Modifies the default unhandled exception behavior.
 *
 * Installs a new terminate() function that, when no catch clauses handle an
 * exception, prints a stacktrace to stderr before performing the default
 * behavior (terminating the process and generating a core file).
 */
void installExceptionHandler();


}


#endif /* BARRETT_EXCEPTION_H_ */
