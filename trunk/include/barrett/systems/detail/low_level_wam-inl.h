/*
 * low_level_wam-inl.h
 *
 *  Created on: Feb 2, 2010
 *      Author: dc
 */


#include <vector>
#include <stdexcept>

#include <libconfig.h++>

#include <barrett/products/puck.h>
#include <barrett/products/low_level_wam.h>


namespace barrett {
namespace systems {


template<size_t DOF>
LowLevelWam<DOF>::LowLevelWam(
		const std::vector<Puck*>& genericPucks,
		Puck* safetyPuck,
		const libconfig::Setting& setting,
		std::vector<int> torqueGroupIds) :
	barrett::LowLevelWam<DOF>(genericPucks, safetyPuck, setting, torqueGroupIds),
	input(sink.input),
	jpOutput(source.jpOutput), jvOutput(source.jvOutput),
	sink(this), source(this)
{
}

template<size_t DOF>
void LowLevelWam<DOF>::Sink::operate()
{
	parent->setTorques(this->input.getValue());
}

template<size_t DOF>
void LowLevelWam<DOF>::Source::operate()
{
	parent->update();
	this->jpOutputValue->setValue(parent->getJointPositions());
	this->jvOutputValue->setValue(parent->getJointVelocities());
}


}
}
