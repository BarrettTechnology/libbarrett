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
	void fail() throw(std::runtime_error);

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
