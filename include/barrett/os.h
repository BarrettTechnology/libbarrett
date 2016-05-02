/*
	Copyright 2009-2014 Barrett Technology <support@barrett.com>

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

*/

/**
 * @file os.h
 * @date Mar 30, 2012
 * @author Dan Cody
 */

#ifndef BARRETT_OS_H_
#define BARRETT_OS_H_


#include <string>
#include <barrett/detail/os.h>


namespace barrett {

/** btsleep and btsleepRT are a functions designed to allow pauses in operation. btsleepRT is real-time safe.
 *
 */
void btsleep(double duration_s);
void btsleepRT(double duration_s);
void btsleep(double duration_s, bool realtime);

/** highResolutionSystemTime maintains the current system time measured in seconds.
 *  The resolution is 1 nanosecond when using Xenomai.
 */
double highResolutionSystemTime();

class PeriodicLoopTimer {
public:
  explicit PeriodicLoopTimer(double period_, const char *treadName, int threadPriority = 10);

	unsigned long wait();

protected:
	bool firstRun;
	double period;
	double releasePoint;
};


/** logMessage function returns an object that can be used in the same way as a boost::format object.
 *   http://www.boost.org/doc/libs/1_49_0/libs/format/
 * The formatted message is output to syslog() and optionally stderr. Use the
 * raise<ExceptionType>() member function to throw an exception passing the
 * formatted message as the "what()" string.
 * Examples:
 *   barrett::logMessage("%s: Error %d", true) % __func__ % 5;
 *   (barrett::logMessage("Bad parameter value: %.2f") % 5.3277).raise<std::runtime_error>();
 */
detail::LogFormatter logMessage(const std::string& message,
		bool outputToStderr = false);


}


#endif /* BARRETT_OS_H_ */
