/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */

/**
 * @file can_socket.h
 * @date 08/18/2010
 * @author Dan Cody
 */

#ifndef BARRETT_BUS_CAN_SOCKET_H_
#define BARRETT_BUS_CAN_SOCKET_H_


#include <stdexcept>

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/real_time_mutex.h>
#include <barrett/bus/abstract/communications_bus.h>


namespace barrett {
namespace bus {


namespace detail {
struct can_handle;  // OS-dependent implementation
}


// TODO(dc): expose a receive timeout option?
class CANSocket : public CommunicationsBus {
public:
	
	static const size_t MAX_MESSAGE_LEN = 8;  /** The maximum length of a CANbus message. Make sure to update CommunicationsBus::MAX_MESSAGE_LEN! */
	
	/** CANSocket() Constructors
	 */
	CANSocket();
	CANSocket(int port) throw(std::runtime_error);
	~CANSocket();
	/** getMutex() method gets and locks interthread data exchange assuring nothing critical is happening in either thread.
	 */
	virtual thread::RealTimeMutex& getMutex() const { return mutex; }
	/** open() method creates socket communication on a specific port.
	 */
	virtual void open(int port) throw(std::logic_error, std::runtime_error);
	/** close() method destorys socket communication port.
	 */
	virtual void close();
	/** isOpen() method returns a flag signifying socket Communication is open.
	 */
	virtual bool isOpen() const;
	/** send() method pushes data onto socket. 
	 */
	virtual int send(int busId, const unsigned char* data, size_t len) const;
	/** receiveRaw() method loads data from socket buffer in a realtime safe manner.
	 */
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking = true) const;

protected:
	mutable thread::RealTimeMutex mutex;
	detail::can_handle* handle;

private:
	DISALLOW_COPY_AND_ASSIGN(CANSocket);
};


}
}


#endif /* BARRETT_BUS_CAN_SOCKET_H_ */
