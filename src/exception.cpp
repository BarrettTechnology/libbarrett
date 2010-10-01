/** <b> Implementation file: do not include.\ </b> Exception handling
 * utilities.
 *
 * @file exception.cpp
 * @date Nov 10, 2009
 * @author Dan Cody
 *
 * @warning
 * This file is located in a \c detail directory. It is part of the
 * implementation and should not be directly included by the user.
 * @see exception.h
 */

/* Copyright 2009 Barrett Technology <support@barrett.com> */

/* This file is part of libbarrett.
 *
 * This version of libbarrett is free software: you can redistribute it
 * and/or modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This version of libbarrett is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this version of libbarrett.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Further, non-binding information about licensing is available at:
 * <http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
 */


#include <exception>
#include <barrett/detail/stacktrace.h>
#include <barrett/exception.h>


namespace barrett {


namespace detail {

void (*oldTerminate)();  // pointer to the system's default terminate function

void myTerminate() {
	print_stacktrace();
	oldTerminate();
}

}


void installExceptionHandler()
{
	detail::oldTerminate = std::set_terminate(detail::myTerminate);
}


}
