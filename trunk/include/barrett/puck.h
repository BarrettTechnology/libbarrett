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
	static const char* getPuckTypeStr(enum PuckType pt) {
		return puckTypeStrs[pt];
	}

// include the generated file containing the list of available properties
#	include <barrett/detail/property_list.h>
	static const char* getPropertyStr(enum Property prop);


public:
	Puck(const CommunicationsBus& bus, int id);
	~Puck();

	void wake();// { wake(bus, id); }

	int getProperty(enum Property prop) const {
		return getProperty(bus, id, getPropertyId(prop));
	}
	int tryGetProperty(enum Property prop, bool* successful,
			int timeout_us = 1000) const {
		return tryGetProperty(bus, id, getPropertyId(prop),
				successful, timeout_us);
	}
	void setProperty(enum Property prop, int value) const {
		setProperty(bus, id, getPropertyId(prop), value);
	}

	bool respondsToProperty(enum Property prop) const {
		return respondsToProperty(prop, effectiveType, vers);
	}
	int getPropertyId(enum Property prop) const throw(std::runtime_error) {
		return getPropertyId(prop, effectiveType, vers);
	}
	int getPropertyIdNoThrow(enum Property prop) const {
		return getPropertyIdNoThrow(prop, effectiveType, vers);
	}

	void updateRole();
	void updateStatus();

	const CommunicationsBus& getBus() const { return bus; }
	int getId() const { return id; }
	int getVers() const { return vers; }
	int getRole() const { return role; }
	enum PuckType getType() const { return type; }
	enum PuckType getEffectiveType() const { return effectiveType; }


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

	static bool respondsToProperty(enum Property prop, enum PuckType pt, int fwVers) {
		return getPropertyIdNoThrow(prop, pt, fwVers) != -1;
	}
	static int getPropertyId(enum Property prop, enum PuckType pt, int fwVers)
			throw(std::runtime_error);
	static int getPropertyIdNoThrow(enum Property prop, enum PuckType pt, int fwVers);


	static const int MIN_ID = 1;
	static const int MAX_ID = 31;
	static const int HOST_ID = 0;  // the Node ID of the control PC
	static const int NODE_ID_WIDTH = 5;
	static const int NODE_ID_MASK = 0x1f;

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

	static const int GROUP_MASK = 0x400;
	static const int FROM_MASK = 0x3e0;
	static const int TO_MASK = 0x41f;

	static const int SET_MASK = 0x80;
	static const int PROPERTY_MASK = 0x7f;

	static const int ROLE_MASK = 0x1f;

	static const int WAKE_UP_TIME = 1000000;  // microseconds

protected:
	// From puck2:PARSE.H
	enum {
		STATUS_RESET, STATUS_ERR, STATUS_READY
	};
	enum {
		ROLE_TATER,
		ROLE_GIMBALS,
		ROLE_SAFETY,
		ROLE_WRAPTOR,
		ROLE_TRIGGER,
		ROLE_BHAND,
		ROLE_FORCE
	};


	const CommunicationsBus& bus;
	int id;
	int vers, role;
	enum PuckType type, effectiveType;

private:
	static int getPropertyHelper(const CommunicationsBus& bus, int id, int propId,
			bool blocking, bool* successful, int timeout_us);

	static const char puckTypeStrs[][12];

	friend class CANSocket;
	friend class PuckGroup;
};


}


// include template definitions
#include <barrett/detail/puck-inl.h>


#endif /* BARRETT_PUCK_H_ */
