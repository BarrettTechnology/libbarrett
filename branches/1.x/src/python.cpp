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


#include <string>

#include <boost/lexical_cast.hpp>
#include <boost/python.hpp>

#include <barrett/bus/can_socket.h>

#include <barrett/products/puck.h>
#include <barrett/products/product_manager.h>

#include <barrett/systems/real_time_execution_manager.h>
#include <barrett/systems/wam.h>

#include "python.h"


using namespace barrett;
using namespace boost::python;


void makeNamespace(const char* name, void(&buildFunction)()) {
	scope s = class_<Namespace, boost::noncopyable>(name, no_init);
	buildFunction();
}


bool isWamActivated(ProductManager& pm) {
	return pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE;
}


BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Wam_gravityCompensate_overloads, gravityCompensate, 0, 1)
template<size_t DOF>
void wrapWam() {
	std::string name = "Wam" + boost::lexical_cast<std::string>(DOF);
	class_<systems::Wam<DOF>, boost::noncopyable>(name.c_str(), no_init)
		.def("gravityCompensate", &systems::Wam<DOF>::gravityCompensate,
				Wam_gravityCompensate_overloads())
	;
}


BOOST_PYTHON_MODULE(libbarrett)
{
	makeNamespace("bus", pythonBusInterface);
	pythonProductsInterface();  // The products sub-folder doesn't correspond to a namespace


	// WARNING! The python wrappers below are experimental. They are partially
	// implemented and are likely to have API changes in the near future.

	class_<systems::RealTimeExecutionManager, boost::noncopyable>("RealTimeExecutionManager", no_init);
	wrapWam<4>();
	wrapWam<7>();

	def("isWamActivated", &isWamActivated);
}
