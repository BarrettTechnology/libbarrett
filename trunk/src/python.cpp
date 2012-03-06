/*
	Copyright 2011, 2012 Barrett Technology <support@barrett.com>

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
 * python.cpp
 *
 *  Created on: Oct 20, 2011
 *      Author: dc
 */


#include <boost/python.hpp>

#include <barrett/bus/can_socket.h>

#include <barrett/products/puck.h>
#include <barrett/products/product_manager.h>

#include <barrett/systems/real_time_execution_manager.h>
#include <barrett/systems/wam.h>


namespace barrett {


bool isWamActivated(ProductManager& pm) {
	return pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE;
}


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Wam4_gravityCompensate_overloads, gravityCompensate, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Wam7_gravityCompensate_overloads, gravityCompensate, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getExecutionManager_overloads, getExecutionManager, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_waitForWam_overloads, waitForWam, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam4_overloads, getWam4, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam7_overloads, getWam7, 0, 2)


BOOST_PYTHON_MODULE(libbarrett)
{
    using namespace boost::python;


    class_<bus::CANSocket, boost::noncopyable>("CANSocket", init<int>())
    	.def("send", &bus::CANSocket::send)
    	.def("receiveRaw", &bus::CANSocket::receiveRaw)
    ;

    class_<Puck>("Puck", init<bus::CANSocket&, int>())
		.def("wake", (void (Puck::*)())&Puck::wake)  // cast to resolve the overload
//    	.def("getProperty", &Puck::getProperty)
    ;


    class_<systems::RealTimeExecutionManager, boost::noncopyable>("RealTimeExecutionManager", no_init)
    ;

    class_<systems::Wam<4>, boost::noncopyable>("Wam4", no_init)
    	.def("gravityCompensate", &systems::Wam<4>::gravityCompensate, Wam4_gravityCompensate_overloads())
    ;

    class_<systems::Wam<7>, boost::noncopyable>("Wam7", no_init)
    	.def("gravityCompensate", &systems::Wam<7>::gravityCompensate, Wam7_gravityCompensate_overloads())
    ;

    def("isWamActivated", &isWamActivated);

    class_<ProductManager, boost::noncopyable>("ProductManager")
    	.def("getExecutionManager", &ProductManager::getExecutionManager, ProductManager_getExecutionManager_overloads()[return_internal_reference<>()])
    	.def("waitForWam", &ProductManager::waitForWam, ProductManager_waitForWam_overloads())
    	.def("wakeAllPucks", &ProductManager::wakeAllPucks)
    	.def("foundWam7", &ProductManager::foundWam7)
    	.def("getWam7", &ProductManager::getWam7, ProductManager_getWam7_overloads()[return_internal_reference<>()])
    	.def("foundWam4", &ProductManager::foundWam4)
    	.def("getWam4", &ProductManager::getWam4, ProductManager_getWam4_overloads()[return_internal_reference<>()])
    ;
}


}
