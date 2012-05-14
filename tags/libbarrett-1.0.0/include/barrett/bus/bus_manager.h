/*
 * bus_manager.h
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#ifndef BARRETT_BUS_BUS_MANAGER_H_
#define BARRETT_BUS_BUS_MANAGER_H_


#include <map>
#include <cstring>

#include <boost/circular_buffer.hpp>

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>


namespace barrett {
namespace bus {


class BusManager : public CommunicationsBus {
public:
	BusManager(CommunicationsBus* bus = NULL);
	BusManager(int port);
	virtual ~BusManager();

	const CommunicationsBus& getUnderlyingBus() const { return *bus; }
	virtual thread::Mutex& getMutex() const { return bus->getMutex(); }

	virtual void open(int port) { bus->open(port); }
	virtual void close() { bus->close(); }
	virtual bool isOpen() { return bus->isOpen(); }

	virtual int send(int busId, const unsigned char* data, size_t len) const
		{ return bus->send(busId, data, len); }
	virtual int receive(int expectedBusId, unsigned char* data, size_t& len,
			bool blocking = true, bool realtime = false) const;
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len,
			bool blocking = true) const
		{ return bus->receiveRaw(busId, data, len, blocking); }

protected:
	int updateBuffers() const;
	void storeMessage(int busId, const unsigned char* data, size_t len) const;
	bool retrieveMessage(int busId, unsigned char* data, size_t& len) const;

	CommunicationsBus* bus;
	bool deleteBus;

private:
	struct Message {
		Message(const unsigned char* d, size_t l) :
			len(l)
		{
			memcpy(data, d, len);
		}

		void copyTo(unsigned char* d, size_t& l) {
			l = len;
			memcpy(d, data, len);
		}

		unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
		size_t len;
	};

	static const size_t MESSAGE_BUFFER_SIZE = 10;
	class MessageBuffer : public boost::circular_buffer<Message> {
	public:
		MessageBuffer() :
			boost::circular_buffer<Message>(MESSAGE_BUFFER_SIZE) {}
	};

	mutable std::map<int, MessageBuffer> messageBuffers;

	DISALLOW_COPY_AND_ASSIGN(BusManager);
};


}
}


#endif /* BARRETT_BUS_BUS_MANAGER_H_ */
