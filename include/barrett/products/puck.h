/*
 * puck.h
 *
 *  Created on: Aug 20, 2010
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_PUCK_H_
#define BARRETT_PRODUCTS_PUCK_H_


#include <stdexcept>
#include <vector>

#include <syslog.h>

#include <barrett/bus/abstract/communications_bus.h>


namespace barrett {


class Puck {

public:
	enum RoleOption {
		RO_MagEncOnSerial = 0x0100,
		RO_MagEncOnHall = 0x0200,
		RO_MagEncOnEnc = 0x400,
		RO_Strain = 0x0800,
		RO_Tact = 0x1000,
		RO_IMU = 0x2000,
		RO_OpticalEncOnEnc = 0x4000
	};
	enum PuckType {
		PT_Monitor, PT_Safety, PT_Motor, PT_ForceTorque, PT_Unknown
	};

	static const char* getPuckTypeStr(enum PuckType pt) {
		return puckTypeStrs[pt];
	}

// include the generated file containing the list of available properties
#	include <barrett/products/detail/property_list.h>
	static const char* getPropertyStr(enum Property prop);
	static enum Property getPropertyEnum(const char* str) throw(std::invalid_argument);
	static enum Property getPropertyEnumNoThrow(const char* str);


public:
	Puck(const bus::CommunicationsBus& bus, int id);
	~Puck();

	void wake();

	int getProperty(enum Property prop, bool realtime = false) const;
	int tryGetProperty(enum Property prop, int* result, int timeout_ns = 1000000) const;

	template<typename Parser> void getProperty(enum Property prop,
			typename Parser::result_type* result, bool realtime = false) const;
	template<typename Parser> int tryGetProperty(enum Property prop, typename Parser::result_type* result, int timeout_ns = 1000000) const;

	void setProperty(enum Property prop, int value, bool blocking = false) const {
		setProperty(bus, id, getPropertyId(prop), value, blocking);
	}

	void saveProperty(enum Property prop) const;
	void resetProperty(enum Property prop) const;

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

	const bus::CommunicationsBus& getBus() const { return bus; }
	int getId() const { return id; }
	int getVers() const { return vers; }
	int getRole() const { return role; }
	bool hasOption(enum RoleOption ro) const { return role & ro; }
	enum PuckType getType() const { return type; }
	enum PuckType getEffectiveType() const { return effectiveType; }


	static void wake(std::vector<Puck*> pucks);

	static int getProperty(const bus::CommunicationsBus& bus, int id, int propId, bool realtime = false);
	template<typename Parser> static void getProperty(
			const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool realtime = false);
	static int tryGetProperty(const bus::CommunicationsBus& bus, int id, int propId,
			int* result, int timeout_ns = 1000000);
	template<typename Parser> static int tryGetProperty(
			const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result,
			int timeout_ns = 1000000);
	static void setProperty(const bus::CommunicationsBus& bus, int id, int propId,
			int value, bool blocking = false);

	static int sendGetPropertyRequest(const bus::CommunicationsBus& bus, int id, int propId);
	static int receiveGetPropertyReply(const bus::CommunicationsBus& bus, int id, int propId,
			int* result, bool blocking, bool realtime);
	template<typename Parser> static int receiveGetPropertyReply(
			const bus::CommunicationsBus& bus, int id, int propId, typename Parser::result_type* result, bool blocking, bool realtime);

	static bool respondsToProperty(enum Property prop, enum PuckType pt, int fwVers) {
		return getPropertyIdNoThrow(prop, pt, fwVers) != -1;
	}
	static int getPropertyId(enum Property prop, enum PuckType pt, int fwVers)
			throw(std::runtime_error);
	static int getPropertyIdNoThrow(enum Property prop, enum PuckType pt, int fwVers);


	static const int DEFAULT_IPNM = 2700;

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

	static const int WAKE_UP_TIME = 1000000000;  // nanoseconds
	static const int TURN_OFF_TIME = 10000000;  // nanoseconds

	struct StandardParser {
		static int busId(int id, int propId);

		typedef int result_type;
		static int parse(int id, int propId, result_type* result, const unsigned char* data, size_t len);
	};


protected:
	// From puck2:PARSE.H
	enum {
		STATUS_RESET, STATUS_ERR, STATUS_READY
	};

	static const int ROLE_MASK = 0x1f;
	enum {
		ROLE_TATER,
		ROLE_GIMBALS,
		ROLE_SAFETY,
		ROLE_WRAPTOR,
		ROLE_TRIGGER,
		ROLE_BHAND,
		ROLE_FORCE
	};


	const bus::CommunicationsBus& bus;
	int id;
	int vers, role;
	enum PuckType type, effectiveType;

private:
	template<typename Parser>
	static int getPropertyHelper(const bus::CommunicationsBus& bus,
			int id, int propId, typename Parser::result_type* result, bool blocking, bool realtime, int timeout_ns);

	static const char puckTypeStrs[][12];
};


}


// include template definitions
#include <barrett/products/detail/puck-inl.h>


#endif /* BARRETT_PRODUCTS_PUCK_H_ */
