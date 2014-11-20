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

/** Defines Communication::Bus
 * 
 * @file bus_manager.h
 * @date 09/18/2010
 * @author Dan Cody
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
	/** BusManager Constructors and Destructors
	 */
	BusManager(CommunicationsBus* bus = NULL);
	BusManager(int port);
	virtual ~BusManager();
	/** getUnderlyingBus pointer returns bus.
	 */
	const CommunicationsBus& getUnderlyingBus() const { return *bus; }
	/**
	 */
	virtual thread::Mutex& getMutex() const { return bus->getMutex(); }
	/** Open Method creates the communication port on CANBus
	 */
	virtual void open(int port) { bus->open(port); }
	/** close Method destroys the communication port on CANBus
	 */
	virtual void close() { bus->close(); }
	/** isOpen Method is flag for available communication on CANBus
	 */
	virtual bool isOpen() const { return bus->isOpen(); }
	/** send Method 
	 */
	virtual int send(int busId, const unsigned char* data, size_t len) const
		{ return bus->send(busId, data, len); }
	/** receive Method is thread safe way to update CANBus messages
	 */
	virtual int receive(int expectedBusId, unsigned char* data, size_t& len,
			bool blocking = true, bool realtime = false) const;
	/** receiveRaw Method works the same as receive but is realtime safe
	 */
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
