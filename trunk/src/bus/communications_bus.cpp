/*
 * communications_bus.cpp
 *
 *  Created on: Sep 2, 2010
 *      Author: dc
 */

#include <stdexcept>

#include <barrett/os.h>
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
		logMessage("CommunicationsBus::%s: Received unexpected message from busId=%d while expecting a message from busId=%d.  Check /var/log/syslog for details.") %__func__ %actualBusId %expectedBusId).raise<std::runtime_error>();
	}

	return 0;
}



}
}
