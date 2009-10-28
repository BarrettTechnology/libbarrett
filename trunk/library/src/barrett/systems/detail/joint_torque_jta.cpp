/*
 * transparent_jta.cpp
 *
 *  Created on: Oct 21, 2009
 *      Author: dc
 */


#include "../../units.h"
#include "../abstract/system.h"
#include "../joint_torque_jta.h"


namespace barrett {
namespace systems {


template<size_t N>
inline System::Input<units::JointTorques<N> >*
	JointTorqueJTA<N>::getControlInput()
{
	return &controlInput;
}

template<size_t N>
inline void JointTorqueJTA<N>::operate()
{
	this->jointTorqueOutputValue->setValue(controlInput.getValue());
}


}
}
