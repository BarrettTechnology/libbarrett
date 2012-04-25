/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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

/*
 * os.cpp
 *
 *  Created on: Mar 28, 2012
 *      Author: dc
 */


#include <stdexcept>
#include <iostream>
#include <cassert>

#include <syslog.h>
#include <native/task.h>

#include <boost/thread.hpp>

#include <barrett/detail/os.h>
#include <barrett/os.h>


namespace barrett {


void btsleep(double duration_s)
{
	assert(duration_s > 1e-6);  // Minimum duration is 1 us
	boost::this_thread::sleep(boost::posix_time::microseconds(long(duration_s * 1e6)));
}

void btsleepRT(double duration_s)
{
	assert(duration_s > 1e-9);  // Minimum duration is 1 ns
	int ret = rt_task_sleep(RTIME(duration_s * 1e9));
	if (ret != 0) {
		(logMessage("%s: rt_task_sleep() returned error %d.") % __func__ % ret).raise<std::runtime_error>();
	}
}

void btsleep(double duration_s, bool realtime)
{
	if (realtime) {
		btsleepRT(duration_s);
	} else {
		btsleep(duration_s);
	}
}



detail::LogFormatter logMessage(const std::string& message, bool outputToStderr)
{
	return detail::LogFormatter(message, outputToStderr);
}


namespace detail {

void LogFormatter::print()
{
	// Make sure we only print once
	if (printed) {
		return;
	}
	printed = true;

	std::string message = str();
	if (ose) {
		std::cerr << message;
		// The message should always have one newline
		if (message[message.size() - 1] != '\n') {
			std::cerr << std::endl;
		}
	}
	// Use a trivial format string in case message contains '%'
	syslog(LOG_ERR, "%s", message.c_str());
}

}
}
