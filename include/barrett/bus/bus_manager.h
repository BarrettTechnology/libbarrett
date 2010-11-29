/*
 * bus_manager.h
 *
 *  Created on: Aug 18, 2010
 *      Author: dc
 */

#ifndef BARRETT_BUS_BUS_MANAGER_H_
#define BARRETT_BUS_BUS_MANAGER_H_


#include <map>
#include <vector>
#include <cstring>

#include <boost/circular_buffer.hpp>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/bus/can_socket.h>
#include <barrett/products/puck.h>
#include <barrett/products/hand.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/force_torque_sensor.h>


namespace barrett {

// Forward declarations
namespace systems{

template<size_t DOF> class Wam;
class ExecutionManager;
class RealTimeExecutionManager;

}


class BusManager : public CommunicationsBus {
public:
	BusManager(const char* configFile = "/etc/barrett/default.conf");
	virtual ~BusManager();

	void enumerate();
	void wakeAllPucks() const { Puck::wake(getPucks()); }

	bool foundSafetyModule() const;
	SafetyModule* getSafetyModule();

	const std::vector<Puck*>& getWamPucks() const;
	bool foundWam() const { return foundWam4() || foundWam7(); }
	bool foundWam4() const;
	bool foundWam7() const;
	bool foundWam7Wrist() const;
	bool foundWam7Gimbals() const;

	void waitForWam(bool promptOnZeroing = true);
	const char* getWamDefaultConfigPath();
	systems::Wam<4>* getWam4(bool waitForShiftActivate = true, const char* configPath = NULL);
	systems::Wam<7>* getWam7(bool waitForShiftActivate = true, const char* configPath = NULL);
	systems::RealTimeExecutionManager* getExecutionManager(double T_s = DEFAULT_LOOP_PERIOD);

	bool foundForceTorqueSensor() const;
	ForceTorqueSensor* getForceTorqueSensor();

	const std::vector<Puck*>& getHandPucks() const;
	bool foundHand() const;
	Hand* getHand();

	const std::vector<Puck*>& getPucks() const { return pucks; }
	Puck* getPuck(int id) const;
	void deletePuck(Puck* p);

//	const CommunicationsBus& getBus() const { return bus; }
	libconfig::Config& getConfig() { return config; }

	virtual thread::Mutex& getMutex() const { return bus.getMutex(); }

	virtual void open(int port) { bus.open(port); }
	virtual void close() { bus.close(); }
	virtual bool isOpen() { return bus.isOpen(); }

	virtual int send(int busId, const unsigned char* data, size_t len) const
		{ return bus.send(busId, data, len); }
	virtual int receive(int expectedBusId, unsigned char* data, size_t& len,
			bool blocking = true, bool realtime = false) const;
	virtual int receiveRaw(int& busId, unsigned char* data, size_t& len,
			bool blocking = true) const
		{ return bus.receiveRaw(busId, data, len, blocking); }


	static const size_t MAX_WAM_DOF = 7;
	static const double DEFAULT_LOOP_PERIOD = 0.002;
	static const int SAFETY_MODULE_ID = 10;
	static const int FIRST_WAM_ID = 1;
	static const int FIRST_HAND_ID = 11;
	static const int FORCE_TORQUE_SENSOR_ID = 8;

protected:
	int updateBuffers() const;
	void storeMessage(int busId, const unsigned char* data, size_t len) const;
	bool retrieveMessage(int busId, unsigned char* data, size_t& len) const;

	bool verifyWamPucks(const size_t dof) const;

	libconfig::Config config;
	CommunicationsBus& bus;
	std::vector<Puck*> pucks;
	std::vector<Puck*> wamPucks;
	std::vector<Puck*> handPucks;

	SafetyModule* sm;
	systems::RealTimeExecutionManager* rtem;
	systems::Wam<4>* wam4;
	systems::Wam<7>* wam7;
	ForceTorqueSensor* fts;
	Hand* hand;

private:
	typedef CANSocket ActualBusType;

	bool wam7FoundHelper(int poles) const;

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

		unsigned char data[ActualBusType::MAX_MESSAGE_LEN];
		size_t len;
	};

	static const size_t MESSAGE_BUFFER_SIZE = 10;
	class MessageBuffer : public boost::circular_buffer<Message> {
	public:
		MessageBuffer() :
			boost::circular_buffer<Message>(MESSAGE_BUFFER_SIZE) {}
	};

	ActualBusType actualBus;
	mutable std::map<int, MessageBuffer> messageBuffers;

	DISALLOW_COPY_AND_ASSIGN(BusManager);
};


}


#endif /* BARRETT_BUS_BUS_MANAGER_H_ */
