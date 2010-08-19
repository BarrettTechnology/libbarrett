/*
 * can_socket.h
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#ifndef CAN_SOCKET_H_
#define CAN_SOCKET_H_


#include <stdexcept>

#include "./detail/ca_macro.h"
#include "./thread/real_time_mutex.h"


namespace barrett {


class CANSocket {
	typedef int handle_type;

public:
	static const size_t MAX_MESSAGE_LEN = 8;

	CANSocket();
	CANSocket(int port) throw(std::runtime_error);
	~CANSocket();

	thread::Mutex& getMutex() { return mutex; };

	void open(int port) throw(std::logic_error, std::runtime_error);
	void close();
	bool isOpen() { return handle != NULL_HANDLE; }

	int send(int id, const unsigned char* data, size_t len) const;
	int receive(int& id, unsigned char* data, size_t& len, bool blocking = true) const;

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


#endif /* CAN_SOCKET_H_ */
