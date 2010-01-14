/*
 * kinematics-inl.h
 *
 *  Created on: Jan 14, 2010
 *      Author: dc
 */

#ifndef KINEMATICSINL_H_
#define KINEMATICSINL_H_


#include <libconfig.h>

#include <barrett/kinematics/kinematics.h>
#include "../../units.h"


namespace barrett {
namespace math {


template<size_t DOF>
Kinematics<DOF>::Kinematics(config_setting_t * config)
{
	bt_kinematics_create(&impl, config, DOF);
}

template<size_t DOF>
Kinematics<DOF>::~Kinematics()
{
	bt_kinematics_destroy(impl);
}

template<size_t DOF>
//void Kinematics<DOF>::eval(const units::JointPositions<DOF>& jp, const units::JointVelocities<DOF>& jv)
void Kinematics<DOF>::eval(const jp_type& jp, const jv_type& jv)
{
	bt_kinematics_eval(impl, jp.asGslVector(), jv.asGslVector());
}

template<size_t DOF>
result_type Kinematics<DOF>::operator() (const boost::tuple<jp_type, jv_type>& jointState)
{
	eval(jointState.get<0>(), jointState.get<1>());
	return units::CartesianPosition(impl->tool->origin_pos);
}


}
}


#endif /* KINEMATICSINL_H_ */
