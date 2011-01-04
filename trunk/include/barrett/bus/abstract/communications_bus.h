/*
 * communications_bus.h
 *
 *  Created on: Aug 26, 2010
 *      Author: dc
 */

#ifndef BARRETT_BUS_ABSTRACT_COMMUNICATIONS_BUS_H_
#define BARRETT_BUS_ABSTRACT_COMMUNICATIONS_BUS_H_


#include <barrett/thread/abstract/mutex.h>


namespace barrett {
namespace bus {


class CommunicationsBus {
public:
	static const size_t MAX_MESSAGE_LEN = 8;  //< The maximum of any of the available communications buses
	static const unsigned long long TIMEOUT = 1000000000ULL;  // nanoseconds

	virtual ~CommunicationsBus() {}

	virtual thread::Mutex& getMutex() const = 0;

	virtual void open(int port) = 0;
	virtual void close() = 0;
	virtual bool isOpen() = 0;

	virtual int send(int busId, const unsigned char* data, size_t len) const = 0;
	virtual int receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking = true, bool realtime = false) const;
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking = true) const = 0;
};


}
}


#endif /* BARRETT_BUS_ABSTRACT_COMMUNICATIONS_BUS_H_ */
