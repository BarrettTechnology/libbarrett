/*
 * bus_manager.h
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#ifndef BUS_MANAGER_H_
#define BUS_MANAGER_H_


#include "../detail/ca_macro.h"
#include "../thread/abstract/mutex.h"
#include "./abstract/communications_bus.h"
#include "./can_socket.h"


namespace barrett {


class BusManager : public CommunicationsBus {
public:
	BusManager(const char* configDir = "/etc/barrett/default.conf");
	virtual ~BusManager();

	void enumerate();

//	const CommunicationsBus& getBus() const { return bus; }

	virtual thread::Mutex& getMutex() const { return bus.getMutex(); }

	virtual void open(int port) { bus.open(port); }
	virtual void close() { bus.close(); }
	virtual bool isOpen() { return bus.isOpen(); }

	virtual int send(int busId, const unsigned char* data, size_t len) const { return bus.send(busId, data, len); }
	virtual int receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking = true) const { return bus.receive(expectedBusId, data, len, blocking); }
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len, bool blocking = true) const { return bus.receiveRaw(busId, data, len, blocking); }


protected:
	CommunicationsBus& bus;

private:
	CANSocket actualBus;
	DISALLOW_COPY_AND_ASSIGN(BusManager);
};


}


#endif /* BUS_MANAGER_H_ */
