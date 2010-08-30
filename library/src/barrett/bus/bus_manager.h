/*
 * bus_manager.h
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#ifndef BUS_MANAGER_H_
#define BUS_MANAGER_H_


#include "../detail/ca_macro.h"
#include "./abstract/communications_bus.h"
#include "./can_socket.h"


namespace barrett {


class BusManager {
public:
	BusManager(const char* configDir = "/etc/barrett/default.conf");
	~BusManager();

	void enumerate();

	const CommunicationsBus& getBus() const { return bus; }

protected:
	CommunicationsBus& bus;

private:
	CANSocket actualBus;
	DISALLOW_COPY_AND_ASSIGN(BusManager);
};


}


#endif /* BUS_MANAGER_H_ */
