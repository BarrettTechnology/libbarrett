/*
 * communications_bus.cpp
 *
 *  Created on: Sep 2, 2010
 *      Author: dc
 */

#include <stdexcept>
#include <syslog.h>

#include <barrett/bus/abstract/communications_bus.h>


namespace barrett {
namespace bus {


int CommunicationsBus::receive(int expectedBusId, unsigned char* data, size_t& len, bool blocking, bool realtime) const {
	int actualBusId;
	int ret = receiveRaw(actualBusId, data, len, blocking);

	if (ret != 0) {
		return ret;
	}

	if (actualBusId != expectedBusId) {
		syslog(LOG_ERR, "CommunicationsBus::receive(): Received a message "
				"from busId=%d while expecting a message from busId=%d.",
				actualBusId, expectedBusId);
		throw std::runtime_error("CommunicationsBus::receive(): Received "
				"unexpected message. Check /var/log/syslog for details.");
	}

	return 0;
}



}
}
