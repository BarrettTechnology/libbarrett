/*
 * puck.h
 *
 *  Created on: Aug 20, 2010
 *      Author: dc
 */

#ifndef PUCK_H_
#define PUCK_H_


#include "./can_socket.h"


namespace barrett {


class Puck {
public:
	Puck(const CANSocket& bus, int id);
	~Puck();

	int getProperty(int property) { return getProperty(bus, id, property); }
	void setProperty(int property, int value) { setProperty(bus, id, property, value); }

	static int getProperty(const CANSocket& bus, int id, int property);
	static void setProperty(const CANSocket& bus, int id, int property, int value);

	static const int MAX_ID = 31;
	static const int HOST_ID = 0;  // the Node ID of the control PC

protected:
	static int nodeId2BusId(int id) { return (id & TO_MASK) | (HOST_ID << NODE_ID_WIDTH); }
	static int busId2NodeId(int id) {return (id & FROM_MASK) >> NODE_ID_WIDTH; }

	static const int NODE_ID_WIDTH = 5;
	static const int GROUP_MASK = 0x400;
	static const int FROM_MASK = 0x3e0;
	static const int TO_MASK = 0x41f;

	static const int SET_MASK = 0x80;
	static const int PROPERTY_MASK = 0x7f;

	const CANSocket& bus;
	int id;
};


}


#endif /* PUCK_H_ */
