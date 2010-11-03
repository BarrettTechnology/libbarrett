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


namespace barrett {
namespace systems {


template<size_t DOF>
LowLevelWamWrapper<DOF>::LowLevelWamWrapper(
		const std::vector<Puck*>& genericPucks,
		Puck* safetyPuck,
		const libconfig::Setting& setting,
		std::vector<int> torqueGroupIds) :
	input(sink.input),
	jpOutput(source.jpOutput), jvOutput(source.jvOutput),
	llw(genericPucks, safetyPuck, setting, torqueGroupIds),
	sink(this), source(this)
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
	parent->llw.update();
	this->jpOutputValue->setValue(parent->llw.getJointPositions());
	this->jvOutputValue->setValue(parent->llw.getJointVelocities());
}


}
}
