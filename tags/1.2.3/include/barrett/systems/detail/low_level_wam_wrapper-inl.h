/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

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
 * low_level_wam_wrapper-inl.h
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */


#include <vector>
#include <stdexcept>

#include <libconfig.h++>

#include <barrett/products/puck.h>
#include <barrett/products/low_level_wam.h>
#include <barrett/products/safety_module.h>


namespace barrett {
namespace systems {


template<size_t DOF>
LowLevelWamWrapper<DOF>::LowLevelWamWrapper(
		ExecutionManager* em,
		const std::vector<Puck*>& genericPucks,
		SafetyModule* safetyModule,
		const libconfig::Setting& setting,
		std::vector<int> torqueGroupIds,
		const std::string& sysName) :
	input(sink.input),
	jpOutput(source.jpOutput), jvOutput(source.jvOutput),
	llw(genericPucks, safetyModule, setting, torqueGroupIds),
	sink(this, em, sysName + "::Sink"), source(this, em, sysName + "::Source")
{
}

template<size_t DOF>
void LowLevelWamWrapper<DOF>::Sink::operate()
{
	parent->llw.setTorques(this->input.getValue());
}

template<size_t DOF>
void LowLevelWamWrapper<DOF>::Source::operate()
{
	try {
		parent->llw.update();
	} catch (const std::runtime_error& e) {
		if (parent->llw.getSafetyModule() != NULL  &&  parent->llw.getSafetyModule()->getMode(true) == SafetyModule::ESTOP) {
			throw ExecutionManagerException("systems::LowLevelWamWrapper::Source::operate(): E-stop! Cannot communicate with Pucks.");
		} else {
			throw;
		}
	}

	this->jpOutputValue->setData( &(parent->llw.getJointPositions()) );
	this->jvOutputValue->setData( &(parent->llw.getJointVelocities()) );
}


}
}
