/*
 * product_manager.h
 *
 *  Created on: Jan 3, 2011
 *      Author: dc
 */

#ifndef BARRETT_PRODUCTS_PRODUCT_MANAGER_H_
#define BARRETT_PRODUCTS_PRODUCT_MANAGER_H_


#include <vector>

#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/thread/abstract/mutex.h>
#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/puck.h>
#include <barrett/products/hand.h>
#include <barrett/products/gimbals_hand_controller.h>
#include <barrett/products/safety_module.h>
#include <barrett/products/force_torque_sensor.h>


namespace barrett {

// Forward declarations
namespace systems{

template<size_t DOF> class Wam;
class ExecutionManager;
class RealTimeExecutionManager;

}


class ProductManager {
public:
	static const char DEFAULT_CONFIG_FILE[];  // = "/etc/barrett/default.conf"

	explicit ProductManager(const char* configFile = NULL, bus::CommunicationsBus* bus = NULL);
	virtual ~ProductManager();

	void enumerate();
	void cleanUpAfterEstop();
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

	systems::RealTimeExecutionManager* getExecutionManager(
			double period_s = DEFAULT_LOOP_PERIOD, int rt_priority = 50);
	void startExecutionManager();

	bool foundForceTorqueSensor() const;
	ForceTorqueSensor* getForceTorqueSensor();

	const std::vector<Puck*>& getHandPucks() const;
	bool foundHand() const;
	Hand* getHand();

	bool foundGimbalsHandController() const;
	GimbalsHandController* getGimbalsHandController();

	const std::vector<Puck*>& getPucks() const { return pucks; }
	Puck* getPuck(int id) const;
	void deletePuck(Puck* p);

	libconfig::Config& getConfig() { return config; }
	const bus::CommunicationsBus& getBus() const { return *bus; }
	virtual thread::Mutex& getMutex() const { return bus->getMutex(); }


	static const size_t MAX_WAM_DOF = 7;
	static const double DEFAULT_LOOP_PERIOD = 0.002;
	static const int SAFETY_MODULE_ID = 10;
	static const int FIRST_WAM_ID = 1;
	static const int FIRST_HAND_ID = 11;
	static const int FORCE_TORQUE_SENSOR_ID = 8;

protected:
	void destroyEstopProducts();
	bool verifyWamPucks(const size_t dof) const;

	libconfig::Config config;
	bus::CommunicationsBus* bus;
	bool deleteBus;
	std::vector<Puck*> pucks;
	std::vector<Puck*> wamPucks;
	std::vector<Puck*> handPucks;

	SafetyModule* sm;
	systems::RealTimeExecutionManager* rtem;
	systems::Wam<4>* wam4;
	systems::Wam<7>* wam7;
	ForceTorqueSensor* fts;
	Hand* hand;
	GimbalsHandController* ghc;

private:
	bool wam7FoundHelper(int poles) const;

	DISALLOW_COPY_AND_ASSIGN(ProductManager);
};


}


#endif /* BARRETT_PRODUCTS_PRODUCT_MANAGER_H_ */
