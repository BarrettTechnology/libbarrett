/*
 * puck.h
 *
 *  Created on: Aug 20, 2010
 *      Author: dc
 */

#ifndef BARRETT_PUCK_H_
#define BARRETT_PUCK_H_


#include <stdexcept>
#include <vector>

#include <syslog.h>

#include <barrett/bus/abstract/communications_bus.h>


namespace barrett {


class Puck {

public:
	enum PuckType {
		PT_Monitor, PT_Safety, PT_Motor, PT_ForceTorque, PT_Unknown
	};

// include the generated file containing the list of available properties
#	include <barrett/detail/property_list.h>

public:
	Puck(const CommunicationsBus& bus, int id, enum PuckType pt = PT_Unknown);
	~Puck();

	void wake();// { wake(bus, id); }

	int getProperty(enum Property prop) const {
		return getProperty(bus, id, getPropertyId(prop));
	}
	int tryGetProperty(enum Property prop, bool* successful,
			int timeout_us = 1000) const {
		return tryGetProperty(bus, id, getPropertyId(prop), successful, timeout_us);
	}
	void setProperty(enum Property prop, int value) const {
		setProperty(bus, id, getPropertyId(prop), value);
	}

	int getPropertyId(enum Property prop) const throw(std::runtime_error) {
		return getPropertyId(prop, effectiveType, vers);
	}
	int getPropertyIdNoThrow(enum Property prop) const {
		return getPropertyIdNoThrow(prop, effectiveType, vers);
	}

	void updateStatus();

	int getId() const { return id; }
	enum PuckType getType() const { return type; }
	enum PuckType getEffectiveType() const { return effectiveType; }
	void setType(enum PuckType pt) {
		if (effectiveType == type) { effectiveType = pt; }
		type = pt;
	}
	int getVers() const { return vers; }


	template<template<typename U, typename = std::allocator<U> > class Container>
	static void wake(Container<Puck*> pucks);

	static int getProperty(const CommunicationsBus& bus, int id, int propId);
	static int tryGetProperty(const CommunicationsBus& bus, int id, int propId,
			bool* successful, int timeout_us = 1000);
	static void setProperty(const CommunicationsBus& bus, int id, int propId,
			int value);

	static int sendGetPropertyRequest(const CommunicationsBus& bus, int id, int propId);
	static int receiveGetPropertyReply(const CommunicationsBus& bus, int id, int propId,
			bool blocking, bool* successful);

	static int getPropertyId(enum Property prop, enum PuckType pt, int fwVers)
			throw(std::runtime_error);
	static int getPropertyIdNoThrow(enum Property prop, enum PuckType pt, int fwVers);

	static const int MIN_ID = 1;
	static const int MAX_ID = 31;
	static const int HOST_ID = 0;  // the Node ID of the control PC

protected:
	static int nodeId2BusId(int id) {
		return (id & TO_MASK) | (HOST_ID << NODE_ID_WIDTH);
	}
	static int busId2NodeId(int busId) {
		return (busId & FROM_MASK) >> NODE_ID_WIDTH;
	}
	static int encodeBusId(int fromId, int toId) {
		return (toId & TO_MASK) | ((fromId & NODE_ID_MASK) << NODE_ID_WIDTH);
	}
	static void decodeBusId(int busId, int* fromId, int* toId) {
		*fromId = (busId & FROM_MASK) >> NODE_ID_WIDTH;
		*toId = busId & TO_MASK;
	}

	enum Status { STATUS_RESET = 0, STATUS_ERR = 1, STATUS_READY = 2 };

	static const int NODE_ID_WIDTH = 5;
	static const int NODE_ID_MASK = 0x1f;
	static const int GROUP_MASK = 0x400;
	static const int FROM_MASK = 0x3e0;
	static const int TO_MASK = 0x41f;

	static const int SET_MASK = 0x80;
	static const int PROPERTY_MASK = 0x7f;

	const CommunicationsBus& bus;
	int id;
	enum PuckType type, effectiveType;
	int vers;

private:
	static int getPropertyHelper(const CommunicationsBus& bus, int id, int propId,
			bool blocking, bool* successful, int timeout_us);

	friend class CANSocket;
};


}


// include template definitions
#include <barrett/detail/puck-inl.h>


#endif /* BARRETT_PUCK_H_ */
