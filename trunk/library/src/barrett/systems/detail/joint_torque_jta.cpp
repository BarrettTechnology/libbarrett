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


inline System::Input<units::JointTorques>* JointTorqueJTA::getControlInput()
{
	return &controlInput;
}

inline void JointTorqueJTA::operate()
{
	this->jointTorqueOutputValue->setValue(controlInput.getValue());
}


}
}
