/*
 * bus_manager.cpp
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <syslog.h>

#include <native/task.h>
#include <native/timer.h>

#include <barrett/detail/stl_utils.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/can_socket.h>
#include <barrett/bus/bus_manager.h>


namespace barrett {
namespace bus {


BusManager::BusManager(CommunicationsBus* _bus) :
	bus(_bus), deleteBus(false), messageBuffers()
{
	if (bus == NULL) {
		bus = new CANSocket;
		deleteBus = true;
	}
}

BusManager::BusManager(int port) :
	bus(NULL), deleteBus(true), messageBuffers()
{
	bus = new CANSocket(port);
}

BusManager::~BusManager()
{
	if (deleteBus) {
		delete bus;
	}
}

int BusManager::receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking, bool realtime) const
{
	thread::Mutex& m = getMutex();
	m.lock();

	RTIME start = rt_timer_read();

	if (retrieveMessage(expectedBusId, data, len)) {
		m.unlock();
		return 0;
	}

	int ret;
	while (true) {
		ret = updateBuffers();
		if (ret != 0) {
			m.unlock();
			return ret;
		} else if (retrieveMessage(expectedBusId, data, len)) {
			m.unlock();
			return 0;
		} else if (!blocking) {
			m.unlock();
			return 1;
		}

		if ((rt_timer_read() - start) > CommunicationsBus::TIMEOUT) {
			m.unlock();
			syslog(LOG_ERR, "BusManager::receive(): timed out");
			return 2;
		}

		if (!realtime) {
			int lc = m.fullUnlock();
//			usleep(100);
			rt_task_sleep(100000);
			m.relock(lc);
		}
	}
}

int BusManager::updateBuffers() const
{
	BARRETT_SCOPED_LOCK(getMutex());

	int busId;
	unsigned char data[CommunicationsBus::MAX_MESSAGE_LEN];
	size_t len;
	int ret;

	// empty the bus' receive buffer
	while (true) {
		ret = receiveRaw(busId, data, len, false);  // non-blocking read
		if (ret == 0) {  // successfully received a message
			storeMessage(busId, data, len);
		} else if (ret == 1) {  // would block
			return 0;
		} else {  // error
			return ret;
		}
	}
}

void BusManager::storeMessage(int busId, const unsigned char* data, size_t len) const
{
	if (messageBuffers[busId].full()) {
		syslog(LOG_ERR, "BusManager::storeMessage(): Buffer overflow. ID = %d", busId);
		throw std::runtime_error("BusManager::storeMessage(): Buffer overflow. Check /var/log/syslog for details.");
	}
	messageBuffers[busId].push_back(Message(data, len));
}

bool BusManager::retrieveMessage(int busId, unsigned char* data, size_t& len) const
{
	if (messageBuffers[busId].empty()) {
		return false;
	}

	messageBuffers[busId].front().copyTo(data, len);
	messageBuffers[busId].pop_front();

	return true;
}


}
}
