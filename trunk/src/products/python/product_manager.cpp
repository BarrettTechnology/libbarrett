/*
	Copyright 2012 Barrett Technology <support@barrett.com>

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
 * product_manager.cpp
 *
 *  Created on: Jun 13, 2012
 *      Author: dc
 */


#include <string>

#include <boost/python.hpp>

#include <barrett/bus/abstract/communications_bus.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/systems/real_time_execution_manager.h>


using namespace barrett;
using namespace boost::python;


// Convert char array to a string, otherwise boost::python only recognizes the
// first character.
const std::string DEFAULT_CONFIG_FILE(ProductManager::DEFAULT_CONFIG_FILE);

const size_t MAX_WAM_DOF = ProductManager::MAX_WAM_DOF;
const double DEFAULT_LOOP_PERIOD = ProductManager::DEFAULT_LOOP_PERIOD;
const int SAFETY_MODULE_ID = ProductManager::SAFETY_MODULE_ID;
const int FIRST_WAM_ID = ProductManager::FIRST_WAM_ID;
const int FIRST_HAND_ID = ProductManager::FIRST_HAND_ID;
const int FORCE_TORQUE_SENSOR_ID = ProductManager::FORCE_TORQUE_SENSOR_ID;


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_waitForWam_overloads, waitForWam, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam4_overloads, getWam4, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam7_overloads, getWam7, 0, 2)

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getExecutionManager_overloads, getExecutionManager, 0, 2)


void pythonProductsProductManagerInterface() {
	class_<ProductManager, boost::noncopyable>("ProductManager")
		.def(init<const char*>())
		.def(init<const char*, bus::CommunicationsBus*>()[with_custodian_and_ward<1,3>()])

		.def_readonly("DEFAULT_CONFIG_FILE", DEFAULT_CONFIG_FILE)

		.def("enumerate", &ProductManager::enumerate)
		.def("cleanUpAfterEstop", &ProductManager::cleanUpAfterEstop)
		.def("wakeAllPucks", &ProductManager::wakeAllPucks)

		.def("foundSafetyModule", &ProductManager::foundSafetyModule)
		.def("getSafetyModule", &ProductManager::getSafetyModule, return_internal_reference<>())

//		.def("getWamPucks", &ProductManager::getWamPucks)  // TODO(dc): Need return_internal_reference<>()?
		.def("foundWam", &ProductManager::foundWam)
		.def("foundWam4", &ProductManager::foundWam4)
		.def("foundWam7", &ProductManager::foundWam7)
		.def("foundWam7Wrist", &ProductManager::foundWam7Wrist)
		.def("foundWam7Gimbals", &ProductManager::foundWam7Gimbals)

		.def("waitForWam", &ProductManager::waitForWam, ProductManager_waitForWam_overloads())
		.def("getWamDefaultConfigPath", &ProductManager::getWamDefaultConfigPath)
		.def("getWam4", &ProductManager::getWam4,
				ProductManager_getWam4_overloads()[return_internal_reference<>()])
		.def("getWam7", &ProductManager::getWam7,
				ProductManager_getWam7_overloads()[return_internal_reference<>()])

		.def("getExecutionManager", &ProductManager::getExecutionManager,
				ProductManager_getExecutionManager_overloads()[return_internal_reference<>()])
		.def("startExecutionManager", &ProductManager::startExecutionManager)

		.def("foundForceTorqueSensor", &ProductManager::foundForceTorqueSensor)
		.def("getForceTorqueSensor", &ProductManager::getForceTorqueSensor, return_internal_reference<>())

//		.def("getHandPucks", &ProductManager::getHandPucks)
		.def("foundHand", &ProductManager::foundHand)
		.def("getHand", &ProductManager::getHand, return_internal_reference<>())

		.def("foundGimbalsHandController", &ProductManager::foundGimbalsHandController)
		.def("getGimbalsHandController", &ProductManager::getGimbalsHandController, return_internal_reference<>())

//		.def("getPucks", &ProductManager::getPucks, return_internal_reference<>())
		.def("getPuck", &ProductManager::getPuck, return_internal_reference<>())
//		.def("deletePuck", &ProductManager::deletePuck)

		.def("getConfig", &ProductManager::getConfig, return_internal_reference<>())
		.def("getBus", &ProductManager::getBus, return_internal_reference<>())
		.def("getMutex", &ProductManager::getMutex, return_internal_reference<>())

		.def_readonly("MAX_WAM_DOF", MAX_WAM_DOF)
		.def_readonly("DEFAULT_LOOP_PERIOD", DEFAULT_LOOP_PERIOD)
		.def_readonly("SAFETY_MODULE_ID", SAFETY_MODULE_ID)
		.def_readonly("FIRST_WAM_ID", FIRST_WAM_ID)
		.def_readonly("FIRST_HAND_ID", FIRST_HAND_ID)
		.def_readonly("FORCE_TORQUE_SENSOR_ID", FORCE_TORQUE_SENSOR_ID)
	;
}
