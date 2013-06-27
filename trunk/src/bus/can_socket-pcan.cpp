/*
	Copyright 2013 Barrett Technology <support@barrett.com>

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
 * can_socket-pcan.cpp
 *
 *  Created on: Jun 27, 2013
 *      Author: dc
 */

#include <stdexcept>
#include <cstdio>
#include <cstring>

#include <errno.h>

#include <libpcan.h>

#include <barrett/os.h>
#include <barrett/thread/real_time_mutex.h>
#include <barrett/products/puck.h>
#include <barrett/bus/can_socket.h>


namespace barrett {
namespace bus {


namespace detail {
struct can_handle {
	typedef HANDLE handle_type;
	static const handle_type NULL_HANDLE;
	handle_type h;

	can_handle() : h(NULL_HANDLE) {}
	bool isValid() const { return h != NULL_HANDLE; }
};
const can_handle::handle_type can_handle::NULL_HANDLE = NULL;
}


CANSocket::CANSocket() :
	mutex(), handle(new detail::can_handle)
{
}

CANSocket::CANSocket(int port) throw(std::runtime_error) :
	mutex(), handle(new detail::can_handle)
{
	open(port);
}

CANSocket::~CANSocket()
{
	close();
	delete handle;
	handle = NULL;
}


void CANSocket::open(int port) throw(std::logic_error, std::runtime_error)
{
	if (isOpen()) {
		(logMessage("CANSocket::%s(): This object is already associated with a CAN port.")
				% __func__).raise<std::logic_error>();
	}

	logMessage("CANSocket::open(%d) using PCAN driver") % port;

	// ...
}

void CANSocket::close()
{
	if (isOpen()) {
		// ...
		handle->h = detail::can_handle::NULL_HANDLE;
	}
}

bool CANSocket::isOpen()
{
	return handle->isValid();
}

int CANSocket::send(int busId, const unsigned char* data, size_t len) const
{
	boost::unique_lock<thread::RealTimeMutex> ul(mutex);

	// ...

	return 0;
}

int CANSocket::receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking) const
{
	BARRETT_SCOPED_LOCK(mutex);

	// ...

	return 0;
}


}
}
