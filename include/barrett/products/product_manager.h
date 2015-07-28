/**
 *	Copyright 2009-2014 Barrett Technology <support@barrett.com>
 *
 *	This file is part of libbarrett.
 *
 *	This version of libbarrett is free software: you can redistribute it
 *	and/or modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, either version 3 of the
 *	License, or (at your option) any later version.
 *
 *	This version of libbarrett is distributed in the hope that it will be
 *	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License along
 *	with this version of libbarrett.  If not, see
 *	<http://www.gnu.org/licenses/>.
 *
 *
 *	Barrett Technology Inc.
 *	73 Chapel Street
 *	Newton, MA 02458
 */

/*
 * @file product_manager.h
 *
 * @date Jan 3, 2011
 * @author D Cody
 * @author JP Hagstrand
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
	static const std::string DEFAULT_CONFIG_FILE;  // = "/etc/barrett/default.conf"

	explicit ProductManager(const char* configFile = NULL, bus::CommunicationsBus* bus = NULL);
	virtual ~ProductManager();

	void enumerate();
	void cleanUpAfterEstop();
	void wakeAllPucks() const { Puck::wake(getPucks()); }

	bool foundSafetyModule() const;
	SafetyModule* getSafetyModule();

	const std::vector<Puck*>& getWamPucks() const;
	bool foundWam() const { return foundWam3() || foundWam4() || foundWam7(); } 
	bool foundWam3() const;
	bool foundWam4() const;
	bool foundWam7() const;
	bool foundWam7Wrist() const;
	bool foundWam7Gimbals() const;

	void waitForWam(bool promptOnZeroing = true);
	const char* getWamDefaultConfigPath();
	systems::Wam<3>* getWam3(bool waitForShiftActivate = true, const char* configPath = NULL);
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
	systems::Wam<3>* wam3;
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
