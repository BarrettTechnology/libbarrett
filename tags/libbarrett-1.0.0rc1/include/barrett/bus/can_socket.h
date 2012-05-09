/*
	Copyright 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * can_socket.h
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#ifndef BARRETT_BUS_CAN_SOCKET_H_
#define BARRETT_BUS_CAN_SOCKET_H_


#include <stdexcept>

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/real_time_mutex.h>
#include <barrett/bus/abstract/communications_bus.h>


namespace barrett {
namespace bus {


// TODO(dc): expose a receive timeout option?
class CANSocket : public CommunicationsBus {
	typedef int handle_type;

public:
	static const size_t MAX_MESSAGE_LEN = 8;  //< The maximum length of a CANbus message. Make sure to update CommunicationsBus::MAX_MESSAGE_LEN!

	CANSocket();
	CANSocket(int port) throw(std::runtime_error);
	~CANSocket();

	virtual thread::RealTimeMutex& getMutex() const { return mutex; }

	virtual void open(int port) throw(std::logic_error, std::runtime_error);
	virtual void close();
	virtual bool isOpen() { return handle != NULL_HANDLE; }

	virtual int send(int busId, const unsigned char* data, size_t len) const;
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking = true) const;

protected:
	mutable thread::RealTimeMutex mutex;

	static const handle_type NULL_HANDLE = -1;
	handle_type handle;

private:
	void init();

	DISALLOW_COPY_AND_ASSIGN(CANSocket);
};


}
}


#endif /* BARRETT_BUS_CAN_SOCKET_H_ */
