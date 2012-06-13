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
 *      Author: robot
 */


#include <boost/python.hpp>

#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/systems/real_time_execution_manager.h>

//#include "../../python.h"


using namespace barrett;
using namespace boost::python;


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getExecutionManager_overloads, getExecutionManager, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_waitForWam_overloads, waitForWam, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam4_overloads, getWam4, 0, 2)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(ProductManager_getWam7_overloads, getWam7, 0, 2)


void pythonProductsProductManagerInterface() {
	class_<ProductManager, boost::noncopyable>("ProductManager")
		.def("getExecutionManager", &ProductManager::getExecutionManager,
				ProductManager_getExecutionManager_overloads()[return_internal_reference<>()])
		.def("waitForWam", &ProductManager::waitForWam, ProductManager_waitForWam_overloads())
		.def("wakeAllPucks", &ProductManager::wakeAllPucks)
		.def("foundWam7", &ProductManager::foundWam7)
		.def("getWam7", &ProductManager::getWam7,
				ProductManager_getWam7_overloads()[return_internal_reference<>()])
		.def("foundWam4", &ProductManager::foundWam4)
		.def("getWam4", &ProductManager::getWam4,
				ProductManager_getWam4_overloads()[return_internal_reference<>()])
	;
}
