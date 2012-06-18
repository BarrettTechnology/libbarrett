/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

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
		(logMessage("CommunicationsBus::%s: Received unexpected message from busId=%d while expecting a message from busId=%d.") %__func__ %actualBusId %expectedBusId).raise<std::runtime_error>();
	}

	return 0;
}



}
}
